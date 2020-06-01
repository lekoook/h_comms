#include "ros/ros.h"
#include "std_msgs/String.h"
#include "subt_communication_broker/subt_communication_client.h"

#include <sstream>

void Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "x2_comms");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);

    subt::CommsClient x2Client = subt::CommsClient("X2");

    x2Client.Bind(&Recv, "X2");

    while (ros::ok())
    {
        std::stringstream ss;
        subt::CommsClient::Neighbor_M neigh;
        neigh = x2Client.Neighbors();

        x2Client.SendTo("I am X2", subt::communication_broker::kBroadcast);

        ROS_INFO("-----------------------------------");
        ROS_INFO("X2's Neighbours:");
        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++)
        { 
          ss << it->first << " : " << it->second.first << "," << it->second.second  << " ";
          ROS_INFO("%s", ss.str().c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
{
    std::stringstream ss;
    ss << "X2 received: " << _data << " - from " << _srcAddress;
    ROS_INFO("%s", ss.str().c_str());
}
