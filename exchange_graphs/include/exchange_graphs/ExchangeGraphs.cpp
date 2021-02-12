#include <algorithm>
#include <iterator>
#include <vector>
#include <unordered_map>
#include "ros/ros.h"
#include "exchange_graphs/GeometryGraphStamped.h"

namespace exchange_graphs {

    /* This robot runs an ExchangeGraphs service. */
    class ExchangeGraphs {
    public:
        typedef uint8 ROBOT;
        typedef std::set<ROBOT> ROBOTS;

        typedef graph_msgs::GeometryGraph GRAPH;
        typedef std::vector<GRAPH> GRAPHS;
        typedef exchange_graphs::GeometryGraphStamped GRAPH_STAMPED;

    private:
        ROBOT robot; /**< This robot. */
        ROBOTS otherRobots; /**< All other robots (not including this->robot). */
        GRAPH graph; /**< Graph that this->robot knows. */

        std::unordered_map<GRAPH, ROBOTS> graphToUnawareRobots;
        std::unordered_map<ROBOT, GRAPHS> robotToUnknownGraphs;

    public:
        ExchangeGraphs(ROBOT robot, ROBOTS otherRobots) : robot(robot), otherRobots(otherRobots) {}

        /* Merge g with this graph.
         * @param g Graph to be merged with this graph.
         */
        void graphMerge(GRAPH g) {
            std::vector<NODE> nodes;
            std::vector<EDGE> edges;

            std::set_union(this->graph.nodes.begin(), this->graph.nodes.end(), g.nodes.begin(), g.nodes.end(), std::back_inserter(nodes));
            std::set_union(this->graph.edges.begin(), this->graph.edges.end(), g.edges.begin(), g.edges.end(), std::back_inserter(edges));

            this->graph.nodes = nodes;
            this->graph.edges = edges;
        }

        /* Set difference.
         * g1 - g2.
         * @param g1 First graph.
         * @param g2 Second graph.
         */
        GRAPH graphDifference(GRAPH g1, GRAPH g2) {
            std::vector<NODE> nodes;
            std::vector<EDGE> edges;

            std::set_difference(g1.nodes.begin(), g1.nodes.end(), g2.nodes.begin(), g2.nodes.end(), std::back_inserter(nodes));
            std::set_difference(g1.edges.begin(), g1.edges.end(), g2.edges.begin(), g2.edges.end(), std::back_inserter(edges));

            GRAPH graph;
            graph.nodes = nodes;
            graph.edges = edges;
            return graph;
        }

        /* Record that a graph was explored by this->robot.
         */
        void recordNewGraph(GRAPH g) {

            /* Learn that graph g exists. */
            graphMerge(g);

            /* otherRobots don't (yet) know about g. */
            this->graphToUnawareRobots[g] = this->otherRobots;
            for (ROBOT otherRobot : this->otherRobots) {
                this->robotToUnknownGraphs[otherRobot].insert(g);
            }
        }

        /* Teach:
         * 1. That graphs that this robot thinks studentRobot doesn't know about, actually exist.
         * 2. For each such graph, the robots that this robot thinks don't know about that graph.
         * @param studentRobot Robot being taught.
         */
        void teach(ROBOT studentRobot) {
            for (GRAPH unknownGraph : this->robotToUnknownGraphs[studentRobot]) {
                /* Teach studentRobot. */
                GRAPH_STAMPED gStamped;
                gStamped.graph = unknownGraph;
                gStamped.robots = std::vector(this->graphToUnawareRobots[unknownGraph].begin(), this->graphToUnawareRobots[unknownGraph].end());
                // TODO send(studentRobot, gStamped);

                /* Record that studentRobot knows about unknownGraph. */
                ROBOTS unawareRobots = this->graphToUnawareRobots[unknownGraph];
                unawareRobots.erase(studentRobot);
                this->graphToUnawareRobots[unknownGraph] = unawareRobots;
            }

            /* Record that studentRobot knows everything this->robot knows.
             */
            this->robotToUnknownGraphs[studentRobot] = {};
        }

        /* Learn:
         * 1. That a graph exists.
         * 2. For that graph, the robots that the teacherRobot thinks don't know about that graph.
         */
        void learn() {
            // TODO received from ROBOT teacherRobot;
            GRAPH_STAMPED learntGraphStamped;
            exchange_graphs::GeometryGraphStamped packed;
            // TODO receive(teacherRobot, learntGraphStamped);
            GRAPH learntGraph = learntGraphStamped.graph;

            /* Learn that learntGraph exists. */
            graphMerge(learntGraph);

            /* Learn that teacherRobot knows learntGraph.
             * For each unknownGraph that this robot thinks teacherRobot doesn't know,
             * keep only nodes of the unknownGraph that are not in learntGraph.
             */
            GRAPHS unknownGraphs = this->robotToUnknownGraphs[teacherRobot];
            for (size_t i = 0; i < unknownGraphs.size(); ++i) {
                unknownGraphs[i] = graphDifference(unknownGraphs[i], learntGraph);
            }

            /* Learn that unawareRobots don't know about learntGraph.
             * This means: Learn that awareRobots know about learntGraph.
             * Only awareRobots change their knowledge state.
             */
            ROBOTS unawareRobots = learntGraphStamped.robots;
            ROBOTS awareRobots;
            std::set_difference(this->otherRobots.begin(), this->otherRobots.end(), unawareRobots.begin(), unawareRobots.end(), std::back_inserter(awareRobots));
            for (ROBOT awareRobot : unawareRobots) {
                GRAPHS unknownGraphs = this->robotToUnknownGraphs[awareRobot];
                for (size_t i = 0; i < unknownGraphs.size(); ++i) {
                    GRAPH graphThatRemainsUnknown = graphDifference(unknownGraphs[i], learntGraph);
                    this->graphToUnawareRobots[unknownGraphs[i]].erase(awareRobot); // awareRobot no longer not-knows about unknownGraphs[i].
                    this->graphToUnawareRobots[graphThatRemainsUnknown].insert(awareRobot); // awareRobot now doesn't know about graphThatRemainsUnknown.
                    unknownGraphs[i] = graphThatRemainsUnknown;
                }
            }
        }

    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "exchange_graphs");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}
