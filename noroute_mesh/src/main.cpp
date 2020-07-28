#include "ros/ros.h"
#include "Node.hpp"

// messages
#include "std_msgs/String.h"
#include "noroute_mesh/neighb.h"
#include "noroute_mesh/neighbArray.h"

// services
#include "noroute_mesh/discover.h"
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

    nomesh(std::string robot) {
        nomesh::name = robot;
        cc = new subt::CommsClient(nomesh::name);
        node = new aodv::Node(0,1,nomesh::name,subt::communication_broker::kBroadcast);
        cc->StartBeaconInterval(ros::Duration(2));
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
        //Bind(handlePacket, address);
    }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        ROS_INFO("[SEND_MAP] Response received.");

        // If the message is not for this robot, check table, send to the other node
        // Node.receive(data, &cc->SendTo)
        // peek at the first byte to identify type of ros message
        cc->SendTo("I am X1", subt::communication_broker::kBroadcast);

        // If the message is for this robot, publish

        //map_pub.publish(recv.map)
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        cc->SendTo("I am X1", subt::communication_broker::kBroadcast);
        return true;
    }

    bool getNeighbour(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res){
        // cc->SendTo("ping", subt::communication_broker::kBroadcast);
        // wait a fixed amount of time

        int num = 0;
        std::string names;
        noroute_mesh::neighbArray a;
        subt::CommsClient::Neighbor_M neigh;
        neigh = cc->Neighbors();
        std::stringstream ss;
        std::string temp;

        // Broken printing format
        // Need to find a way to compare time taken with sim time to create a current list of neighbours
        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++){ 
            if (num == 0){
                ss << it->first;
            }
            else{
                ss << it->first << "|";
            }
            temp = ss.str();
            names.append(temp);
            num++;
            // noroute_mesh::neighb temp;
            // temp.name = it->second.second;
            //a.data.insert(temp);
        }
        res.num_neighbours = num;
        res.response = names;
        return true;
    }
};

int main(int argc, char** argv)
{
    if (argc != 2){
        printf("Enter robot name as the argument for this CommsClient.\n");
        return 0;
    }
    std::string robo_name = argv[1];
    ros::init(argc, argv, robo_name + "_nomesh_API");
    nomesh robot(robo_name);
    ros::spin();
}
