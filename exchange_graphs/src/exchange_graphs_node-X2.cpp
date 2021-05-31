#include <ros/ros.h>
#include <exchange_graphs/ExchangeGraphs.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "exchange_graphs");
    exchange_graphs::ExchangeGraphs exchangeGraphs(2, {{"X1", 1}, {"X2", 2}, {"X3", 3}});
    ros::spin();
    return 0;
}
