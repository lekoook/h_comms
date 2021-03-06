#include "ros/ros.h"
#include <json.hpp>
#include "string"
#include "aodv/send_map.h"
#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

bool sendMap(aodv::send_map::Request &req, aodv::send_map::Response &res);
void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);

std::string name = "X3";
int msg_id = 0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "comms_client_node");
    ros::NodeHandle n;
    subt::CommsClient cc = subt::CommsClient(name);

    ros::ServiceServer map_service = n.advertiseService("send_map",sendMap);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid",1);
    cc.Bind(handlePacket, name);
    ROS_INFO("This node receives information from Node.cpp and transmits it to the network");
    ROS_INFO("also receives information from the network and transmits it back to Node.cpp");

    ROS_INFO("Started comms node");

    // Clear queue of messages that need to be transmitted.
    ros::spin();

}

// When there is a message to be sent out from the robot
bool sendMap(aodv::send_map::Request &req, aodv::send_map::Response &res){
    ROS_INFO("[SEND_MAP] Requested");
    // Serialise map and send to network
    nav_msgs::OccupancyGrid map;
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
    // send _data to Node.cpp
    
}
