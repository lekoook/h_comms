#ifndef SIM_H_
#define SIM_H_

#include "../node/Node.hpp"
#include <stdint.h>
#include <queue>

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

  /**
   * @brief An array of event.
   */
  typedef struct config_t {
    int numEvents;
    event_t events[];
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
   * |       |             |
   * v       v             v
   * (App1)  (App2)  ...   (AppN)   }} --> (stdout)
   * |    ^  |    ^        |    ^
   * v    |  v    |        v    |   }} Node::send_app, Node::receive_app
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
     * @brief array of sending queues, one for each node.
     */
    std::queue<Eth> sends[LIM_NODES];
  
    /**
     * @brief array of receiving queues, one for each node.
     */
    std::queue<Eth> recvs[LIM_NODES];
  
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
