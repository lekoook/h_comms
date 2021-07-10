#include <ros/ros.h>
#include <exchange_graphs/ExchangeGraphs.hpp>

int main(int argc, char **argv) {
    exchange_graphs::ROBOT robot = std::atoi(argv[1]);
    ros::init(argc, argv, "exchange_graphs");
    exchange_graphs::ExchangeGraphs exchangeGraphs(robot, {{"X1", 1}, {"X2", 2}, {"X3", 3}});
    ros::spin();
    return 0;
}
