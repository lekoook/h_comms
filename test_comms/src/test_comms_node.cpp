#include "ros/ros.h"
#include "ptp_comms/PtpClient.hpp"
#include <thread>
#include <string>
#include <vector>

std::atomic<bool> toRun(true);

ptp_comms::PtpClient* pptp;
ros::Time start, end;
std::string dest;
std::string name;
std::vector<uint8_t> initial;
std::vector<uint8_t> large;
std::vector<uint8_t> small1;
std::vector<uint8_t> small2;
std::vector<uint8_t> small3;
std::vector<uint8_t> small4;
std::vector<uint8_t> small5;

int largeCnt = 0;

void cb(std::string src, uint16_t port, std::vector<uint8_t> data)
{
    char first = data[0];
    switch (first)
    {
    case 'i':
        ROS_INFO("initial");
        if (name == "X2")
        {
            pptp->sendTo(dest, initial);
        }
        break;
    case 'L':
        ROS_INFO("Large");
        if (name == "X2")
        {
            pptp->sendTo(dest, large);
        }
        largeCnt++;
        break;
    case '1':
        ROS_INFO("One");
        if (name == "X2")
        {
            pptp->sendTo(dest, small1);
        }
        break;
    case '2':
        ROS_INFO("Two");
        if (name == "X2")
        {
            pptp->sendTo(dest, small2);
        }
        break;
    case '3':
        ROS_INFO("Three");
        if (name == "X2")
        {
            pptp->sendTo(dest, small3);
        }
        break;
    case '4':
        ROS_INFO("Four");
        if (name == "X2")
        {
            pptp->sendTo(dest, small4);
        }
        break;
    case '5':
        ROS_INFO("Five");
        if (name == "X2")
        {
            pptp->sendTo(dest, small5);
        }
        break;
    default:
        ROS_INFO("not sure");
    }

    if (name == "X1" && largeCnt == 4)
    {
        end = ros::Time::now();
        auto diff = end - start;

        std::ostringstream oss;
        oss << "Total time taken: " << diff << std::endl;
        ROS_INFO("%s", oss.str().c_str());
    }
}

void run()
{
    auto ptp = ptp_comms::PtpClient(1234);
    pptp = &ptp;
    ptp.bind(std::function<void(std::string, uint16_t, std::vector<uint8_t>)>(cb));
    name = ros::this_node::getNamespace().substr(1);
    for (int i = 0; i < 50000; i++) initial.push_back('i');
    for (int i = 0; i < 1000000; i++) large.push_back('L');
    for (int i = 0; i < 200; i++) small1.push_back('1');
    for (int i = 0; i < 200; i++) small2.push_back('2');
    for (int i = 0; i < 300; i++) small3.push_back('3');
    for (int i = 0; i < 400; i++) small4.push_back('4');
    for (int i = 0; i < 500; i++) small5.push_back('5');
    if (name == "X1")
    {
        dest = "X2";
    }
    else if (name == "X2")
    {
        dest = "X1";
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (name == "X1")
    {
        ROS_INFO("start");
        start = ros::Time::now();
        // ptp.sendTo(dest, initial);
        // ptp.sendTo(dest, initial);
        // ptp.sendTo(dest, initial);
        // ptp.sendTo(dest, initial);
        // ptp.sendTo(dest, initial);
        ptp.sendTo(dest, large);
        ptp.sendTo(dest, large);
        ptp.sendTo(dest, large);
        ptp.sendTo(dest, large);
        // ptp.sendTo(dest, small1);
        // ptp.sendTo(dest, small2);
        // ptp.sendTo(dest, small3);
        // ptp.sendTo(dest, small4);
        // ptp.sendTo(dest, small5);
    }

    while (toRun)
    {
        ros::Rate(1).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_comms");
    ros::NodeHandle nh;

    std::thread th(run);
    
    ros::spin();
    toRun = false;

    th.join();
}