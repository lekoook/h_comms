#include <ros/ros.h>
#include <exchange_graphs/ExchangeGraphs.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "exchange_graphs");
    exchange_graphs::NEIGHBOR_TO_ROBOT neighborToRobot = {{"X1", 1}, {"X2", 2}, {"X3", 3}};
    exchange_graphs::ROBOT robot = neighborToRobot[argv[1]];
    exchange_graphs::ExchangeGraphs exchangeGraphs(robot, neighborToRobot);
    ros::spin();
    return 0;
}
