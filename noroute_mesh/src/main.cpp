#include "iostream"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "Node.hpp"
#include "Eth.hpp"

// messages
#include "std_msgs/String.h"
#include "noroute_mesh/neighb.h"
#include "noroute_mesh/neighbArray.h"

// services
#include "noroute_mesh/send_map.h"
#include "noroute_mesh/neighbour.h"
#include "noroute_mesh/map_to_string.h"
#include "noroute_mesh/string_to_map.h"

#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

void handlePacket2(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);
void string_to_uint8(uint8_t b[], std::string s);
std::string uint8_to_string(uint8_t b[], std::string::size_type l);

class nomesh {
public:

    std::string name;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer map_service, neighbour_service, discovery_service;
    ros::ServiceClient map_to_string_client, string_to_map_client;
    nav_msgs::OccupancyGrid grid;
    
    aodv::Node *node;
    subt::CommsClient *cc;

    nomesh(std::string robotName, uint32_t robotId) {
        nomesh::name = robotName;
        cc = new subt::CommsClient(nomesh::name);
        node = new aodv::Node(robotId, nomesh::name, subt::communication_broker::kBroadcast);
        cc->StartBeaconInterval(ros::Duration(PING_INTERVAL));
        start_server();
    }

    ~nomesh(){
        delete cc;
        delete node;
    }

    void start_server(){
        map_service = nh.advertiseService(nomesh::name + "/send_map",&nomesh::sendMap,this);
        neighbour_service = nh.advertiseService(nomesh::name + "/get_neighbour",&nomesh::getNeighbour,this);
        ROS_INFO("Send map and neighbour service advertised");

        map_to_string_client = nh.serviceClient<noroute_mesh::map_to_string>("map_to_string");
        string_to_map_client = nh.serviceClient<noroute_mesh::string_to_map>("string_to_map");
        ROS_INFO("ROS messages conversion service subscribed");

        pub = nh.advertise<nav_msgs::OccupancyGrid>(nomesh::name + "/comms_publisher",1);
        nomesh::cc->Bind(std::bind(&nomesh::handlePacket,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), this->name, 4100);
    }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        tl::optional<aodv::Eth> e = nomesh::node->receive(_data, cc);
        if (e)
        {
            ROS_INFO("Received map from source %s", e.value().src.c_str());
            noroute_mesh::string_to_map srv;
            srv.request.str.data = e.value().payload;
            if (string_to_map_client.call(srv))
            {
                pub.publish(srv.response.grid);
            }
            else
            {
                ROS_ERROR("Failed to call service: string_to_map");
            }
           
        }
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        aodv::Eth e;
        
        e.dstLength = req.dest.length();
        e.dst = req.dest;
        e.srcLength = nomesh::name.length();
        e.src = nomesh::name;

        noroute_mesh::map_to_string srv;
        srv.request.grid = req.grid;
        if (map_to_string_client.call(srv))
        {
            e.payload = srv.response.str.data;
            e.payloadLength = e.payload.size();
            nomesh::node->send(e, this->cc, true);
            ROS_INFO("Sent serialised payload of size %u bytes", e.payloadLength);
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service: map_to_string");
            return false;
        }
    }

    bool getNeighbour(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res){
        int num = 0;
        subt::CommsClient::Neighbor_M neigh = cc->Neighbors();
        std::stringstream ss;

        // Get current simulation time to compare with.
        double currentTime = ros::Time::now().toSec();

        // Filter out neighbours that are outdated.
        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++) {
            if (it->first == this->name)
            {
                continue; // ignore ourselves.
            }
            
            double neighTime = it->second.first;
            if (currentTime - neighTime <= NEIGHBOUR_TIME_THRESH)
            {
                if (num > 0)
                {
                    ss << "|"; // delimiter
                }
                ss << it->first << "," << it->second.second; // print Name and RSSI
                ROS_INFO("%lf", it->second.second);
                num++;
            }
        }
        res.num_neighbours = num;
        res.response = ss.str();
        return true;
    }

private:
    const double NEIGHBOUR_TIME_THRESH = 3.0;
    const double PING_INTERVAL = 1.0;
};

int main(int argc, char** argv)
{
    if (argc != 3){
        printf("Enter robot name and id as the arguments for this CommsClient.\n");
        printf("Eg: [rosrun ...] RobotA 1\n");
        return 0;
    }
    std::string robotName = argv[1];
    std::string robotIdStr = argv[2];
    uint32_t robotId = stoi(robotIdStr);
    ros::init(argc, argv, robotName + "_nomesh_API", ros::init_options::AnonymousName);
    nomesh robot(robotName, robotId);
    ros::spin();
    // nav_msgs::OccupancyGrid grid;
    // while (ros::ok()){
    //     if (robot.payload_flag){
    //         ros::serialization::IStream stream(&robot.payload[0], robot.payload.size());
    //         ros::serialization::deserialize(stream,grid);
    //         robot.payload_flag = false;
    //     }
    //     ros::spinOnce();
    // }
}

std::string uint8_to_string(uint8_t b[], std::string::size_type l){
    /*
    * A uint8_t has bits denoted as: abcdefgh.
    */
    char c;
    uint8_t s[l*2 + 1]; // + 1 for '\0'
    std::string::size_type i=0;
    for (; i<l; i++) {
        c = 0b10101010u;
        c |= (b[i] >> 7) << 6;
        c |= ((b[i] >> 6) & 1) << 4;
        c |= ((b[i] >> 5) & 1) << 2;
        c |= (b[i] >> 4) & 1;
        s[2*i] = c;
        c = 0b10101010u;
        c |= ((b[i] >> 3) & 1) << 6;
        c |= ((b[i] >> 2) & 1) << 4;
        c |= ((b[i] >> 1) & 1) << 2;
        c |= b[i] & 1;
        s[2*i+1] = c;
    }
    s[2*i] = '\0';
    return std::string((char*)s,l*2);
}

void string_to_uint8(uint8_t b[], std::string s)
{
    /*
        * Denotation of bits is as in the body of uint8_to_string(uint8_t b[], std::string::size_type l).
        */
    for (std::string::size_type i=0; i<s.size(); i+=2) {
        b[i/2] = ((s[i] & 0b01000000u) << 1) | ((s[i] & 0b00010000u) << 2) | ((s[i] & 0b00000100u) << 3) | ((s[i] & 1) << 4);
        b[i/2] |= ((s[i+1] & 0b01000000u) >> 3) | ((s[i+1] & 0b00010000u) >> 2) | ((s[i+1] & 0b00000100u) >> 1) | (s[i+1] & 1);
        }
    }