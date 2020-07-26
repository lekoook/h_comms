#include "ros/ros.h"
#include <json.hpp>
#include "string"
#include "node/Node.hpp"
#include "noroute_mesh/send_map.h"
#include "noroute_mesh/neighbour.h"
#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res);
bool getNeighbours(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res);
void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);

std::string name = "X3";
std::stringstream neighbours;
int msg_id = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nomesh_API");
    ros::NodeHandle n;
    subt::CommsClient cc = subt::CommsClient(name);

    //ros::ServiceServer neighbour_service = n.advertiseService("get_neighbours",getNeighbours);
    ros::ServiceServer map_service = n.advertiseService("send_map",sendMap);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid",1);
    cc.Bind(handlePacket, name);
    ROS_INFO("Started nomesh node");

    while (ros::ok()){
        // Updates neighbours state
        subt::CommsClient::Neighbor_M neigh = cc.Neighbors();
        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++){ 
          neighbours << it->first << " : " << it->second.first << "," << it->second.second;
        }
    }
    ros::spin();
}

// When there is a message to be sent out from the robot
bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
    ROS_INFO("[SEND_MAP] Requested");
    // Serialise map and send to AODV layer
    nav_msgs::OccupancyGrid map;
    uint32_t serial_size = ros::serialization::serializationLength(map);


    std::string dest = "X2";
    int ttl = 10;

    nlohmann::json obj;
    obj["ttl"] = ttl;
    obj["id"]= msg_id++;
    obj["src"] = name;
    obj["dest"] = dest;

    std::string serialisedData = obj.dump();
    //cc.SendTo(serialisedData,subt::communication_broker::kBroadcast);
    ROS_INFO("[SEND_MAP] Response: %d", res.response);
}

void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
    // Deserialise the received data
    ROS_INFO("[SEND_MAP] Response received.");
    //mapObject recv = (mapObject) _data;
    // Identify type of packet (e.g RREP, RREQ)
    
    // If the message is not for this robot, check table, send to the other node

    // If the message is for this robot, publish
    //map_pub.publish(recv.map)
}

bool getNeighbours(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res){
    ROS_INFO("[GET_NEIGHBOUR] Requested");
    res.response = neighbours.str();
    ROS_INFO("[GET_NEIGHBOUR] Response: %d", res.response);
}
