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

#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

class nomesh {
public:

    std::string name;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer map_service, neighbour_service, discovery_service;
    
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
    }

    void start_server(){
        map_service = nh.advertiseService(nomesh::name + "/send_map",&nomesh::sendMap,this);
        neighbour_service = nh.advertiseService(nomesh::name + "/get_neighbour",&nomesh::getNeighbour,this);
        pub = nh.advertise<std_msgs::String>("comms_publisher",1);
        
        std::string message = nomesh::name + ": Service server is up.";
        ROS_INFO("Service server is up");
        // nomesh::cc->Bind(nomesh::handlePacket, nomesh::name,this);
    }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        ROS_INFO("[SEND_MAP] Response received.");
        //nomesh::node->receive(_data,&nomesh::cc->SendTo,this);

        // If the message is for this robot, publish
        // peek at the first byte to identify type of ros message

        // uint32_t serial_size = ros::serialization::serializationLength(my_value);
        // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        // ser::IStream stream(buffer.get(), serial_size);
        // ser::deserialize(stream, my_value);

        // pub.publish(recv.map)
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        //cc->SendTo("I am X1", subt::communication_broker::kBroadcast);
        aodv::Eth e;
        e.dstLength = req.dest.length();
        e.dst = req.dest;
        e.srcLength = nomesh::name.length();
        e.src = nomesh::name;

        uint16_t serial_size = ros::serialization::serializationLength(req.grid);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::serialize(stream, req.grid);

        e.payloadLength = serial_size;
        e.payload = buffer.get();
        nomesh::node->send(e, this->cc);
        // nomesh::node->send(e,&send);
        return true;
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
    ros::init(argc, argv, robotName + "_nomesh_API");
    nomesh robot(robotName, robotId);
    ros::spin();
}
