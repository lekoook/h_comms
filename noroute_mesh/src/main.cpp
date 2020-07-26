#include "ros/ros.h"
//#include "node/Node.hpp"
#include "noroute_mesh/neighb.h"
#include "noroute_mesh/neighbArray.h"
#include "noroute_mesh/send_map.h"
#include "noroute_mesh/neighbour.h"
#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

class nomesh {
public:

    std::string name;
    ros::NodeHandle nh;
    ros::ServiceServer map_service, neighbour_service;

    // Trying to figure out how to have a variable name
    subt::CommsClient cc = subt::CommsClient("X1");

    nomesh(std::string robot) {
        // nomesh::name = robot;
        start_server();
    }

    void start_server(){
        map_service = nh.advertiseService("send_map",&nomesh::sendMap,this);
        neighbour_service = nh.advertiseService("get_neighbour",&nomesh::getNeighbour,this);
        ROS_INFO("Service server is up");
    }

    // void discovery(){
    //     while (ros::ok()){
    //         cc.SendTo("ping", subt::communication_broker::kBroadcast);
    //     }
    // }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Deserialise the received data
        ROS_INFO("[SEND_MAP] Response received.");

        // If the message is not for this robot, check table, send to the other node
        cc.SendTo("I am X1", subt::communication_broker::kBroadcast);

        // If the message is for this robot, publish
        //map_pub.publish(recv.map)
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        cc.SendTo("I am X1", subt::communication_broker::kBroadcast);
        return true;
    }

    bool getNeighbour(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res){
        int num = 0;
        std::string names;
        noroute_mesh::neighbArray a;
        subt::CommsClient::Neighbor_M neigh;
        neigh = cc.Neighbors();

        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++){ 
            num++;
            std::stringstream ss;
            std::string temp;
            ss << it->first << "|";
            temp = ss.str();
            names.append(temp);
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
    ros::init(argc, argv, "nomesh_API");
    nomesh robot("X1");
    ros::spin();
}
