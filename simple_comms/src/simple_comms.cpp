#include "ros/ros.h"
#include "std_msgs/String.h"
#include "subt_communication_broker/subt_communication_client.h"
#include "comms_time.h"
#include <sstream>

void x1Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);
void x2Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_comms");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    // Comms client for 'X1' robot.
    subt::CommsClient x1Client = subt::CommsClient("X1");
    // Comms client for 'X2' robot.
    subt::CommsClient x2Client = subt::CommsClient("X2");

    /*
    Uncomment below for unicast.
    // Bind all messages destined for 'X1' to a callback function.
    */
    x1Client.Bind(&x1Recv, "X1"); // Subscribe to any messages intended for 'X1' which is itself.

    /*
    Uncomment below for broadcast.
    */
    // x1Client.Bind(&x1Recv, "X1"); // Same as unicast example.
    // x2Client.Bind(&x2Recv, "X2"); // Same as unicast example.

    /* 
    Uncomment below for multicast.
    NOTE: cannot get this to work.
    */
    // x1Client.Bind(&x1Recv, subt::communication_broker::kMulticast); // Subscribe to the one and only multicast group.
    // x2Client.Bind(&x2Recv, "X2"); // Same as unicast example.

    /*
    Uncomment below for neighbour discovery.
    */
    // x1Client.Bind(&x1Recv, "X1");
    // x2Client.Bind(&x2Recv, "X2");
    // x2Client.StartBeaconInterval(ros::Duration(1));

    while (ros::ok())
    {
        ROS_INFO("time now is - %lu ms", h_comms::time_ms());
        /*
        Uncomment below for unicast.
        */
        // x2Client.SendTo("hello world", "X1");

        /*
        Uncomment below for broadcast.
        */
        // x2Client.SendTo("this is a broadcast", subt::communication_broker::kBroadcast); // Will also broadcast to itself.
        
        /*
        Uncomment below for multicast.
        */
        // x2Client.SendTo("this is a multicast", "kMulticast"); // Send to 'X1' only.



        /*
        Report any neighbours seen.
        */
        // subt::CommsClient::Neighbor_M neigh1 = x1Client.Neighbors();
        // ROS_INFO("-------------");
        // ROS_INFO("X1's Neighbours:");
        // for (subt::CommsClient::Neighbor_M::iterator it = neigh1.begin(); it != neigh1.end(); it++)
        // { 
        //   std::stringstream ss;
        //   ss << it->first << " : " << it->second.first << "," << it->second.second;
        //   ROS_INFO("%s", ss.str().c_str());
        // }
        // ROS_INFO("-------------");

        // ROS_INFO("^^^^^^^^^^^^^");

        // subt::CommsClient::Neighbor_M neigh2 = x2Client.Neighbors();
        // ROS_INFO("-------------");
        // ROS_INFO("X2's Neighbours:");
        // for (subt::CommsClient::Neighbor_M::iterator it = neigh2.begin(); it != neigh2.end(); it++)
        // { 
        //   std::stringstream ss;
        //   ss << it->first << " : " << it->second.first << "," << it->second.second;
        //   ROS_INFO("%s", ss.str().c_str());
        // }
        // ROS_INFO("-------------");
        // ROS_INFO("             ");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// X1 callback
void x1Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
{
    std::stringstream ss;
    ss << "X1 received: " << _data << " - from " << _srcAddress;
    ROS_INFO("%s", ss.str().c_str());
}

// X2 callback
void x2Recv(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
{
    std::stringstream ss;
    ss << "X2 received: " << _data << " - from " << _srcAddress;
    ROS_INFO("%s", ss.str().c_str());
}