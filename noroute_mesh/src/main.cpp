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

#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

void handlePacket2(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);

class nomesh {
public:

    std::string name;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer map_service, neighbour_service, discovery_service;
    nav_msgs::OccupancyGrid grid;
    
    aodv::Node *node;
    subt::CommsClient *cc;
    std::vector<uint8_t> payload;
    bool payload_flag = false;

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
        pub = nh.advertise<nav_msgs::OccupancyGrid>(nomesh::name + "/comms_publisher",1);

        std::cout << cc->Host() << std::endl;
        std::string message = nomesh::name + ": Service server is up.";
        ROS_INFO("Service server is up");
        //nomesh::cc->Bind(&handlePacket2, this->name);
        nomesh::cc->Bind(std::bind(&nomesh::handlePacket,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), this->name, 4100);
    }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        std::cout << "SourceAddress: " << _srcAddress << std::endl
            << "DestAddress: "<<_dstAddress << std::endl
            << "DestPort: "<< _dstPort << std::endl;
        ROS_INFO("[SEND_MAP] Response received.");
        
        tl::optional<aodv::Eth> e = nomesh::node->receive(_data, cc);

        if (e){
            std::cout << "Source: " << e.value().src << std::endl;
            std::cout << "HANDLE PACKET" << std::endl
                << e.value().seq << std::endl
                << e.value().dst << std::endl
                << e.value().dstLength << std::endl
                << e.value().src << std::endl
                << e.value().srcLength << std::endl
                << e.value().payloadLength << std::endl
                << e.value().payload.size() << std::endl;
            nomesh::payload = e.value().payload;
            nomesh::payload_flag = true;

            // uint8_t buffer[e.value().payloadLength];
            
            boost::shared_array<uint8_t> buffer(new uint8_t[e.value().payloadLength]);
            for (int j=0; j<e.value().payloadLength; j++) {
                buffer[j] = e.value().payload[j];
            }

            ros::serialization::IStream stream(buffer.get(), e.value().payloadLength);
            ros::serialization::deserialize(stream, nomesh::grid);
            ros::serialization::Serializer<nav_msgs::OccupancyGrid>::read(stream, nomesh::grid);
            pub.publish(nomesh::grid);
        }
        // If the message is for this robot, publish
        // peek at the first byte to identify type of ros message
        
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        aodv::Eth e;
        // std::cout << nomesh::name << "Sendmap called" << std::endl;
        // cc->SendTo("Hello","X2");
        e.dstLength = req.dest.length();
        e.dst = req.dest;
        e.srcLength = nomesh::name.length();
        e.src = nomesh::name;

        uint16_t serial_size = ros::serialization::serializationLength(req.grid);
        uint8_t *buffer = new uint8_t[serial_size];
        ros::serialization::OStream stream(buffer, serial_size);
        ros::serialization::serialize(stream, req.grid);

        std::cout << "Serial size: " << serial_size << std::endl;
        e.payloadLength = serial_size;
        //std::cout << "Array size: " << sizeof(buffer) << std::endl;
        std::vector<uint8_t> p(&buffer[0],&buffer[serial_size]);
        std::cout << "Vector size: " << p.size() << std::endl;
        e.payload = p;

        nav_msgs::OccupancyGrid grid;
        ros::serialization::IStream stream2(&(e.payload)[0], e.payloadLength);
        ros::serialization::deserialize(stream2, grid);

        pub.publish(grid);

        std::cout << nomesh::name << "Sendmap called" << std::endl;
        nomesh::node->send(e, this->cc, true);
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

void handlePacket2(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        ROS_INFO("[SEND_MAP] Response received.");
        std::cout << "SourceAddress: " << _srcAddress << std::endl
            << "DestAddress: "<<_dstAddress << std::endl
            << "DestPort: "<< _dstPort << std::endl;
        
        // tl::optional<aodv::Eth> e = nomesh::node->receive(_data, cc);

        // if (e){
        //     std::cout << "Source: " << e.value().src << std::endl;
        // }
        // If the message is for this robot, publish
        // peek at the first byte to identify type of ros message

        // uint32_t serial_size = ros::serialization::serializationLength(my_value);
        // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        // ser::IStream stream(buffer.get(), serial_size);
        // ser::deserialize(stream, my_value);

        // pub.publish(recv.map)
    }