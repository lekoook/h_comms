#include <exchange_graphs/ExchangeGraphs.hpp>

int main(int argc, char **argv) {
    exchange_graphs::NEIGHBOR_TO_ROBOT neighborToRobot = {{"X1", 1}, {"X2", 2}, {"X3", 3}};
    for (NEIGHBOR_TO_ROBOT::iterator it = neighborToRobot.begin(); it != m.end(); ++it) {
        exchange_graphs::ROBOT robot = it->second;
        exchange_graphs::ExchangeGraphs exchangeGraphs = exchange_graphs::ExchangeGraphs(robot, neighborToRobot);
    }
    ros::init(argc, argv, "exchange_graphs");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}
