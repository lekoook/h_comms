#include "Node.hpp"
#include <stdint.h>

namespace aodv
{
    Node::Node() :
        table(), seq(), id(), addr(), rreqStack(), send_app(), receive_app()
    {
    }

    Node::Node(Table table, uint32_t seq, uint32_t id, uint32_t addr, void (*send_app)(Eth eth), Eth (*receive_app)()) :
        table(table), seq(seq), id(id), addr(addr), send_app(send_app), receive_app(receive_app)
    {
    }

    void Node::send(Eth eth, void (*f)(uint8_t* msg))
        /* TODO
         * if (eth.dst == this->addr) {
         *   if (is data packet) {
         *     send to application.
         *   }
         * }
         */

        /* RFC3561: section 6.1 */
        // TODO prepare RREQ
        this->seq++;
        // TODO originate RREQ

        /* RFC3561: section 6.4 */
        /* TODO
        ttl = TTL_START;
        while (ttl < TTL_THRESHOLD) {
            uint8_t timeout = RING_TRAVERSAL_TIME;
            // TODO wait for RREP
            ttl += TTL_INCREMENT;
        }
        ttl = NET_DIAMETER;
        // TODO wait for RREP
        */

        // TODO serialize eth to uint8_t* data, and send over link
    }

    void receive(uint8_t* (*f)())
    {
        aodv::Eth eth;
        aodv_msgs::Rreq rreq;
        aodv_msgs::Rrep rrep;
        aodv_msgs::Rerr rerr;
        aodv_msgs::RrepAck rrepAck;
        Route row;
        bool isInvalid;
        uint64_t lifetime;

        uint8_t* data = f();
        eth.deserialise(data);
        uint8_t* payload = eth.payload;
        uint8_t ttl = *payload; // assume TTL is head of payload // TODO do something with ttl
        uint8_t* msg = payload + 1;
        aodv_msgs::MsgTypes t = msg_peeker::peekType(msg);

        /* RFC3561: section 6.1 */
        // In order to ascertain that information about a destination is not stale, the node compares its current numerical value for the sequence number with that obtained from the incoming AODV message.  This comparison MUST be done using signed 32-bit arithmetic, this is necessary to accomplish sequence number rollover.
        // If the result of subtracting the currently stored sequence number from the value of the incoming sequence number is less than zero, then the information related to that destination in the AODV message MUST be discarded.
        if (t == aodv_msgs::MsgTypes::Rreq)
        {
            rreq.deserialise(msg);
            if ((int)rreq.destSeq - (int)this->seq < 0) {
                return;
            }
        }
        else if (t == aodv_msgs::MsgTypes::Rrep)
        {
            rrep.deserialise(msg);
            if ((int)rrep.destSeq - (int)this->seq < 0) {
                return;
            }
        }
        else if (t == aodv_msgs::MsgTypes::Rerr)
        {
            rerr.deserialise(msg);
            if ((int)rerr.destSeq - (int)this->seq < 0) {
                return;
            }
        }
        
        if (t == aodv_msgs::MsgTypes::Rreq)
        {
            /* RFC3561: section 6.2 */
            if (rreq.destAddr == this->addr) {
                /* RFC3561: section 6.1 */
                // TODO prepare RREP
                this->seq = std::max(this->seq, rreq.destSeq);
                // TODO originate RREP

            } else {
                int isearch = table.rsearch(rreq.destAddr);
                if (isearch > 0) { // update route to destination
                    uint16_t i = isearch;
                    row = table.rread(i);
                    if (rreq.destSeq > row.destSeq || ((rreq.destSeq > row.destSeq) && (rreq.hopCount+1 < row.hopCount))) { // TODO || sequence number is unknown
                        isInvalid = false; // TODO check if sequence number is invalid
                        if (isInvalid) {
                            /* RFC3561: section 6.4 */
                            /* TODO
                            ttl = rreq.hopCount + TTL_INCREMENT;
                            while (ttl < TTL_THRESHOLD) {
                                uint8_t timeout = RING_TRAVERSAL_TIME;
                                // TODO wait for RREP
                                ttl += TTL_INCREMENT;
                            }
                            ttl = NET_DIAMETER;
                            uint8_t timeout = NET_TRAVERSAL_TIME;
                            // TODO wait for RREP
                            */
                        }
                        // TODO get nextHop
                        // TODO get precursors from Rreq
                        lifetime = ACTIVE_ROUTE_TIMEOUT;
                        table.rupdate(i, Route(rreq.destAddr, rreq.destSeq, isInvalid, rreq.hopCount, nextHop, precursors, lifetime));
                    }

                } else { // create route to destination
                    isInvalid = false; // TODO check if sequence number is invalid
                    // TODO get nextHop
                    // TODO get precursors from Rreq
                    lifetime = ACTIVE_ROUTE_TIMEOUT;
                    table.rcreate(Route(rreq.destAddr, rreq.destSeq, isInvalid, rreq.hopCount, nextHop, precursors, lifetime));

                    /* RFC3561: section 6.3 */
                    // TODO set unknown sequence number flag if destination sequence number is unknown
                    rreq.srcSeq = this->seq;
                    rreq.id = this->id + 1;
                    rreq.hopCount = 0;

                    // TODO buffer this->id and this->addr for PATH_DISCOVERY_TIME
                    // TODO [belongs elsewhere] any generation of a RREP by an intermediate node (as in section 6.6) for delivery to the originating node SHOULD be accompanied by some action that notifies the destination about a route back to the originating node. The originating node selects this mode of operation in the intermediate nodes by setting the ’G’ flag.

                    // TODO wait for RREP, if route not received within some time, broadcast another RREQ satisfying some conditions
                    // TODO drop data packets destined for destination from FIFO buffer if no RREP received
                    // TODO deliver Destination Unreachable message to application
                    // TODO wait for RREP, if route not received within some time, broadcast another RREQ with binary exponential backoff

                }
                // TODO [belongs elsewhere] each time a route is used to forward a data packet, update lifetime for all nodes in active route to current_time+ACTIVE_ROUTE_TIMEOUT

            }

            /* RFC3561: section 6.5 */
            /* TODO
            std::time_t std::time(0)
            rreq->srcAddr
            rreq->id
            rreqStack.push();
            */
        }

        else if (t == aodv_msgs::MsgTypes::Rrep)
        {
            /* RFC3561: section 6.1 */
            /* TODO
             * A node may change the sequence number in the routing table entry of a destination only if:
             * -  it is itself the destination node, and offers a new route to itself, or
             * -  it receives an AODV message with new information about the sequence number for a destination node, or
             * -  TODO [belongs elsewhere] the path towards the destination node expires or breaks.
             */

            /* RFC3561: section 6.2 */
            // TODO same as section 6.2 for Rreq
        }

        else if (t == aodv_msgs::MsgTypes::Rerr)
        {
            /* RFC3561: section 6.1 */
            // in response to a lost or expired link to the next hop towards that destination.
            // TODO The node determines which destinations use a particular next hop by consulting its routing table.  In this case, for each destination that uses the next hop, the node increments the sequence number and marks the route as invalid (see also sections 6.11, 6.12).
        }

        else if (t == aodv_msgs::MsgTypes::RrepAck)
        {
        }

        else // TODO not a control packet, must be a data packet.
        {
        }

    }

    void Node::send_data(Eth eth, callbackres_t (*cb)(Eth eth)) {
        // TODO check if dest exists in routing table. Else, send RREQ and wait for RREP.
        /* TODO
         * if (RREP timeout) {
         *     return {true, Eth()};
         * } else {
         *     return {false, eth};
         * }
         */
    }
}
