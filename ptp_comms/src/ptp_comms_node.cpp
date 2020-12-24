#include "ros/ros.h"
#include "PTP.hpp"

int main(int argc, char** argv)
{
    std::string robotAddr = argv[1];

    ros::init(argc, argv, "point_to_point_comms");
    ros::NodeHandle nh;

    ptp_comms::PTP ptp(nh, robotAddr);

    ros::spin();
}