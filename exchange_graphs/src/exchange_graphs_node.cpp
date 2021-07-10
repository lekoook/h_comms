#include <ros/ros.h>
#include <exchange_graphs/ExchangeGraphs.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "exchange_graphs");
    exchange_graphs::NEIGHBOR_TO_ROBOT neighborToRobot = {{"X1", 1}, {"X2", 2}, {"X3", 3}};
    exchange_graphs::ROBOT robot = neighborToRobot[argv[1]];
    exchange_graphs::ExchangeGraphs exchangeGraphs(robot, neighborToRobot);

    graph_msgs::Edges edge;
    edge.node_ids = {0};
    graph_msgs::GeometryGraph graph;
    graph.edges = {edge};
    graph.explored = {0};
    /*
     * robot 1 records graph (1,1,1).
     * robot 2 records graph (2,2,2).
     * robot 3 records graph (3,3,3).
     */
    geometry_msgs::Point node;
    node.x = robot;
    node.y = robot;
    node.z = robot;
    graph.nodes = {node};
    exchangeGraphs.recordNewGraph(graph);
    std::thread ros_info_Th = std::thread([&]() {
            while (true) {
            printf("# ROBOT %d . graph\n", robot);
            exchangeGraphs.ros_info_GRAPH(exchangeGraphs.graph);
            ros::Duration(10.0).sleep();
            }
            });
    ros::spin();
    if (ros_info_Th.joinable()) {
        ros_info_Th.join();
    }
    return 0;
}
