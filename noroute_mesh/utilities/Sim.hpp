#ifndef SIM_H_
#define SIM_H_

#include "../node/Node.hpp"
#include <stdint.h>
#include <queue>
#include <string>
#include <iostream>
#include <algorithm>

namespace aodv
{

    /**
     * @brief Defined for both app sending to node, and app receiving from node.
     * Sending: app triggers an event originating at the node with @param src of @param eth.
     * Receiving: node with @param src of @param eth sends @param eth back to app.
     */
    typedef struct event_t {
        int time;
        Eth eth;
    } event_t;

    const int LIM_EVENTS = 256;

    /**
     * @brief An array of event.
     */
    typedef struct config_t {
        int numEvents;
        event_t events[LIM_EVENTS];
    } config_t;

    /**
     * @brief Interface to mesh topology of nodes.
     *
     * __Architecture__:
     *                       (stdin)
     *                       |
     *                       v
     * (node configurations) (commands)
     * |                     |
     * v                     v
     * (Sim                           )
     * (App1)  (App2)  ...   (AppN)   }} --> (stdout)
     * |    ^  |    ^        |    ^   }} made up of |[Node::fifoFromApp]    ^[Node::fifoToApp]
     * v    |  v    |        v    |   }}            v                       |
     * (Node1) (Node2) ...   (NodeN)
     * |    ^  |    ^        |    ^   }} made up of |[sends]    ^[recvs]
     * v    |  v    |        v    |   }}            v           |
     * \/\/\/\/\/\/\/\MESH/\/\/\/\/\
     *
     * The node configurations tell the Sim which @param time to activate events at.
     */
    class Sim
    {
        const static int LIM_NODES = 256;

    private:
        /**
         * @brief simulation time.
         */
        int time = 0;

        /**
         * @brief number of nodes in simulation.
         */
        uint8_t numNodes = 1;
  
        /**
         * @brief array of nodes in simulation.
         * Each element is a node.
         */
        Node nodes[LIM_NODES];
  
        /**
         * @brief sending queues, one for each node.
         */
        std::unordered_map<std::string, std::queue<Eth>> sends;
  
        /**
         * @brief receiving queues, one for each node.
         */
        std::unordered_map<std::string, std::queue<Eth>> recvs;
  
        /**
         * @brief Queue of events, ordered by time.
         */
        std::queue<event_t> events = std::queue<event_t>();

    public:
        /**
         * @brief Construct a new object.
         * Initialize queue of events, ordered by time.
         * 
         * @param numNodes number of nodes to initialize.
         * @param configs configurations for each node.
         */
        Sim(uint8_t numNodes, config_t configs[]);

        /**
         * @brief Pass a command into the simulation.
         *
         * @param command command.
         */
        void command(std::string command);

        /**
         * @brief Start the simulation.
         * Read commands from stdin.
         */
        void start();

        /**
         * @brief Print nodes[i], recvs[i], sends[i].
         *
         * @param i node index.
         */
        void print_node(uint8_t i);

        /**
         * @brief Print state of sends[i].
         *
         * @param i node index.
         */
        void print_send(uint8_t i);

        /**
         * @brief Print state of recvs[i].
         *
         * @param i node index.
         */
        void print_recv(uint8_t i);

        /**
         * @brief Print dst, addr, length.
         *
         * @param eth Eth.
         */
        void print_eth(Eth eth);
    };
}

#endif // SIM_H_
