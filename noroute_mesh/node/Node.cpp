#include "Node.hpp"
#include <stdint.h>

namespace aodv
{
    Node::Node()
    {
    }

    Node::Node(uint32_t seq, uint32_t id, uint32_t addr, void (*send_app)(Eth eth), Eth (*receive_app)()) :
        seq(seq), id(id), addr(addr)//, send_app(send_app), receive_app(receive_app)
    {
        // TODO initialise the function pointers properly
    }

    void Node::originate_payload(uint8_t dst, uint16_t length, uint8_t* payload, void (*send_link)(uint8_t* msg))
    {
        aodv::Eth eth = aodv::Eth(this->seq, dst, this->addr, length, payload);
        uint8_t msg[aodv::ETH_NONPAYLOAD_LEN + eth.length];
        eth.serialise(msg);
        send_link(msg);
    }

    void Node::send(Eth eth, void (*send_link)(uint8_t* msg))
    {
        if (eth.dst == this->addr) {
            /* TODO
             *   if (is data packet) {
             *     send to application.
             *   }
             */

        } else {
            // Packet not meant for this node, forward to next node.
            if ( // eth.dst exists in routing table
                 0) {
                // TODO
            } else {
                // eth.dst does not exist in routing table
            }

        }
    }

    void Node::receive(uint8_t* (*receive_link)(), void (*send_link)(uint8_t* msg))
    {
        aodv::Eth eth;

        uint8_t* data = receive_link();
        eth.deserialise(data);
        uint8_t* payload = eth.payload;
    }
}
