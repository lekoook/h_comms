#include <algorithm>
#include <iterator>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <string>
#include <ros/ros.h>
#include <ptp_comms/PtpClient.hpp>
#include <ptp_comms/Neighbors.h>
#include <graph_msgs/GeometryGraph.h>
#include <geometry_msgs/Point.h>
#include <rosmsg_compressor/rosmsg_serializer.h>
#include "messages/GraphMsg.hpp"
#include "messages/AdvMsg.hpp"

namespace exchange_graphs {

    typedef uint8_t ROBOT;
    typedef std::set<ROBOT> ROBOTS;

    typedef graph_msgs::GeometryGraph GRAPH;
    typedef geometry_msgs::Point NODE;
    typedef graph_msgs::Edges EDGE;
    typedef std::vector<GRAPH> GRAPHS;
    typedef std::vector<NODE> NODES;
    typedef std::vector<EDGE> EDGES;

    typedef std::map<ptp_comms::Neighbor, ROBOT> NEIGHBOR_TO_ROBOT;

    /** This robot runs an ExchangeGraphs service. */
    class ExchangeGraphs {
    private:
        ROBOT robot; /**< This robot. */
        ROBOTS otherRobots; /**< All other robots (not including this->robot). */
        GRAPH graph; /**< Graph that this->robot knows. */

        struct GraphComparator {
            bool operator()(GRAPH a, GRAPH b) {
                NODES A = a.nodes;
                NODES B = b.nodes;
                auto nodesComparator = [](NODE a, NODE b){return a.x<b.x && a.y<b.y && a.z<b.z;};
                std::sort(A.begin(), A.end(), nodesComparator);
                std::sort(B.begin(), B.end(), nodesComparator);
                return std::includes(A.begin(), A.end(), B.begin(), B.end(), nodesComparator);
            }
        };
        std::map<GRAPH, ROBOTS, GraphComparator> graphToUnawareRobots;
        std::unordered_map<ROBOT, GRAPHS> robotToUnknownGraphs;

        std::unique_ptr<ptp_comms::PtpClient> ptpClient;

        NEIGHBOR_TO_ROBOT neighborToRobot; /**< Map from neighbor to robot. */

        const uint16_t PORT = 62478; /**< Port number to use. 62478 looks like "GRAPH". */
        const double ADV_INTERVAL = 5.0; /**< Amount of seconds to sleep before advertising again. */

    public:
        ExchangeGraphs(ROBOT robot, NEIGHBOR_TO_ROBOT neighborToRobot) : robot(robot), neighborToRobot(neighborToRobot) {
            for (NEIGHBOR_TO_ROBOT::iterator it = this->neighborToRobot.begin(); it != this->neighborToRobot.end(); ++it) {
                if (this->robot != it->second) {
                    this->otherRobots.insert(it->second);
                }
            }
            this->ptpClient = std::unique_ptr<ptp_comms::PtpClient>(new ptp_comms::PtpClient(this->PORT, true));
            this->ptpClient->bind(std::bind(&ExchangeGraphs::handleRx, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            this->advertiseLoop();
        }

        /**
         * @brief Destroy the object.
         *
         */
        ~ExchangeGraphs() {
            this->ptpClient->unregister();
        }

        /** Merge g with this graph.
         * @param g Graph to be merged with this graph.
         */
        void graphMerge(GRAPH g) {
            // Increase each index of each edge in g.edges.
            for (size_t i = 0; i < g.edges.size(); ++i) {
                EDGE e = g.edges[i];
                std::vector<uint32_t> node_ids = e.node_ids;
                for (size_t j = 0; j < node_ids.size(); ++j) {
                    node_ids[j] += this->graph.nodes.size();
                }
                e.node_ids = node_ids;
                g.edges[i] = e;
            }

            this->graph.nodes.insert(this->graph.nodes.end(), g.nodes.begin(), g.nodes.end());
            this->graph.edges.insert(this->graph.edges.end(), g.edges.begin(), g.edges.end());
        }

        /** Graph difference.
         * g1 - g2.
         * @param g1 First graph.
         * @param g2 Second graph.
         */
        GRAPH graphDifference(GRAPH g1, GRAPH g2) {
            NODES nodes;
            EDGES edges;

            EDGES::iterator edgesIt = g1.edges.begin();
            for (NODES::iterator nodesIt = g1.nodes.begin(); nodesIt != g1.nodes.end(); ) {
                bool found = false;
                for (NODE n : g2.nodes) {
                    if ((*nodesIt).x == n.x && (*nodesIt).y == n.y && (*nodesIt).z == n.z) {
                        found = true;
                    }
                }
                if (found) {
                    nodesIt = g1.nodes.erase(nodesIt);
                    edgesIt = g1.edges.erase(edgesIt);
                } else {
                    ++nodesIt;
                    ++edgesIt;
                }
            }

            GRAPH graph;
            graph.nodes = nodes;
            graph.edges = edges;
            return graph;
        }

        /** Record that a graph was explored by this->robot.
         */
        void recordNewGraph(GRAPH g) {

            /* Learn that graph g exists. */
            this->graphMerge(g);

            /* otherRobots don't (yet) know about g. */
            this->graphToUnawareRobots[g] = this->otherRobots;
            for (ROBOT otherRobot : this->otherRobots) {
                this->robotToUnknownGraphs[otherRobot].push_back(g);
            }
        }

        /** Teach:
         * 1. That graphs that this robot thinks studentRobot doesn't know about, actually exist.
         * 2. For each such graph, the robots that this robot thinks don't know about that graph.
         * @param studentRobot Robot being taught.
         */
        void teach(ROBOT studentRobot) {
            for (GRAPH unknownGraph : this->robotToUnknownGraphs[studentRobot]) {
                std::vector<ROBOT> robots(this->graphToUnawareRobots[unknownGraph].begin(), this->graphToUnawareRobots[unknownGraph].end());
                /* Teach studentRobot. */
                GraphMsg::GRAPH_STAMPED gStamped;
                gStamped.graph = unknownGraph;
                gStamped.robots = robots;
                // Find Neighbor corresponding to studentRobot.
                ptp_comms::Neighbor neighbor;
                GraphMsg msg = GraphMsg(gStamped);
                for (NEIGHBOR_TO_ROBOT::iterator it = this->neighborToRobot.begin(); it != this->neighborToRobot.end(); ++it) {
                    if (studentRobot == it->second) {
                        this->transmit(it->first, msg);
                    }
                }

                /* Record that studentRobot knows about unknownGraph. */
                ROBOTS unawareRobots = this->graphToUnawareRobots[unknownGraph];
                unawareRobots.erase(studentRobot);
                this->graphToUnawareRobots[unknownGraph] = unawareRobots;
            }

            /* Record that studentRobot knows everything this->robot knows.
             */
            this->robotToUnknownGraphs[studentRobot] = {};
        }

        /** Learn:
         * 1. That a graph exists.
         * 2. For that graph, the robots that the teacherRobot thinks don't know about that graph.
         */
        void learn(ROBOT teacherRobot, GraphMsg::GRAPH_STAMPED learntGraphStamped) {
            GRAPH learntGraph = learntGraphStamped.graph;

            /* Learn that learntGraph exists. */
            this->graphMerge(learntGraph);

            /* Learn that teacherRobot knows learntGraph.
             * For each unknownGraph that this robot thinks teacherRobot doesn't know,
             * keep only nodes of the unknownGraph that are not in learntGraph.
             */
            GRAPHS unknownGraphs = this->robotToUnknownGraphs[teacherRobot];
            for (size_t i = 0; i < unknownGraphs.size(); ++i) {
                unknownGraphs[i] = this->graphDifference(unknownGraphs[i], learntGraph);
            }

            /* Learn that unawareRobots don't know about learntGraph.
             * This means: Learn that awareRobots know about learntGraph.
             * Only awareRobots change their knowledge state.
             */
            std::vector<ROBOT> unawareRobots = learntGraphStamped.robots;
            std::vector<ROBOT> awareRobots;
            std::set_difference(this->otherRobots.begin(), this->otherRobots.end(), unawareRobots.begin(), unawareRobots.end(), std::back_inserter(awareRobots));
            for (ROBOT awareRobot : awareRobots) {
                GRAPHS unknownGraphs = this->robotToUnknownGraphs[awareRobot];
                for (size_t i = 0; i < unknownGraphs.size(); ++i) {
                    GRAPH graphThatRemainsUnknown = this->graphDifference(unknownGraphs[i], learntGraph);
                    this->graphToUnawareRobots[unknownGraphs[i]].erase(awareRobot); // awareRobot no longer not-knows about unknownGraphs[i].
                    this->graphToUnawareRobots[graphThatRemainsUnknown].insert(awareRobot); // awareRobot now doesn't know about graphThatRemainsUnknown.
                    unknownGraphs[i] = graphThatRemainsUnknown;
                }
            }
        }

        /**
         * Advertise forever.
         */
        void advertiseLoop() {
            while (true) {
                AdvMsg msg = AdvMsg();
                this->transmit(ptp_comms::BROADCAST_ADDR, msg);
                ros::Duration(this->ADV_INTERVAL).sleep();
            }
        }

        /**
         * @brief Transmits message by calling the data transmission service.
         *
         * @param dest Intended recipient of data.
         * @param msg Message to transmit.
         * @return true If call was successful.
         * @return false If call has failed.
         */
        bool transmit(std::string dest, BaseMsg& msg)
        {
            std::vector<uint8_t> data = msg.serialize();
            return this->ptpClient->sendTo(dest, data);
        }

        /**
         * @brief Peeks at the type of message the bytes vector contains.
         *
         * @param data Bytes vector.
         * @return MsgType Type of message.
         */
        MsgType peekType(std::vector<uint8_t>& data) {
            return (MsgType)data.data()[0];
        }

        /**
         * Handle received messages by dispatching to the correct handler.
         */
        void handleRx(ptp_comms::Neighbor neighbor, uint16_t port, std::vector<uint8_t> data) {
            ROBOT robot = this->neighborToRobot[neighbor];
            MsgType t = this->peekType(data);
            switch (t) {
                case MsgType::Graph:
                    {
                        GraphMsg msg;
                        msg.deserialize(data);
                        this->learn(robot, msg.learntGraphStamped);
                        break;
                    }
                case MsgType::Advertisement:
                    {
                        this->teach(robot);
                        break;
                    }
                default:
                    {
                        std::ostringstream oss;
                        oss << "Unknown message type: " << t << std::endl;
                        ROS_ERROR("%s", oss.str().c_str());
                        break;
                    }
            }
        }

    };

}
