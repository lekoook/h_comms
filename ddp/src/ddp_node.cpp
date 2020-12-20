#include "ros/ros.h"
#include "DDP.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_distribution_protocol");
    ros::NodeHandle nh;

    DDP ddp(nh);
    
    ros::spin();
}