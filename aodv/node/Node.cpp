#include "Node.hpp"
#include <stdint.h>

namespace aodv
{
    Node::Node()
    {
    }

    Node::Node(Table table, uint32_t seq, uint32_t id, uint32_t addr, uint8_t precursors[], void (*send_app)(Eth eth), Eth (*receive_app)()) :
        table(table), seq(seq), id(id), addr(addr), precursors()//, send_app(send_app), receive_app(receive_app)
    {
        // TODO initialise the function pointers properly
    }

    void Node::send(Eth eth, void (*f)(uint8_t* msg))
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

                /* RFC3561: section 6.1 */

                // prepare RREQ
                aodv_msgs::Rreq rreq = aodv_msgs::Rreq();
                rreq.destAddr = eth.dst;
                rreq.srcAddr = eth.src;

                // MUST increment node sequence number
                this->seq++;

                // originate RREQ
                // construct eth2 with rreq as payload
                // eth2.dst is nextHop.
                uint8_t ttl = 0; //TODO
                uint8_t dst = 0; // TODO
                uint16_t length = aodv_msgs::RREQ_LEN;
                uint8_t payload[length];
                rreq.serialise(payload);
                aodv::Eth eth2 = aodv::Eth(ttl, dst, this->addr, length, payload);
                uint8_t msg[aodv::ETH_NONPAYLOAD_LEN + eth2.length];
                eth2.serialise(msg);
                f(msg);
            }

            /* RFC3561: section 6.4 */
            // When it is desired to have all retries traverse the entire ad hoc network, this can be achieved by configuring TTL_START and TTL_INCREMENT both to be the same value as NET_DIAMETER.
            eth.ttl = TTL_START;
            while (eth.ttl < TTL_THRESHOLD) {
                // TODO broadcast RREQ
                const uint8_t RING_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * (eth.ttl + TIMEOUT_BUFFER);
                uint16_t timeout = RING_TRAVERSAL_TIME;
                // TODO wait for RREP
                eth.ttl += TTL_INCREMENT;
            }
            eth.ttl = NET_DIAMETER;
            // TODO wait for RREP

            // TODO serialize eth to uint8_t* data, and send over link
        }
    }

    void Node::receive(uint8_t* (*f)())
    {
        aodv::Eth eth;
        aodv_msgs::Rreq rreq;
        aodv_msgs::Rrep rrep;
        aodv_msgs::Rerr rerr;
        aodv_msgs::RrepAck rrepAck;
        Route row;
        bool validSequenceNumber;
        uint64_t lifetime;

        uint8_t* data = f();
        eth.deserialise(data);
        uint8_t* payload = eth.payload;
        aodv_msgs::MsgTypes t = msg_peeker::peekType(payload);

        /* RFC3561: section 6.1 */
        // In order to ascertain that information about a destination is not stale, the node compares its current numerical value for the sequence number with that obtained from the incoming AODV message.  This comparison MUST be done using signed 32-bit arithmetic, this is necessary to accomplish sequence number rollover.
        // If the result of subtracting the currently stored sequence number from the value of the incoming sequence number is less than zero, then the information related to that destination in the AODV message MUST be discarded.
        if (t == aodv_msgs::MsgTypes::Rreq)
        {
            rreq.deserialise(payload);
            if ((int)rreq.destSeq - (int)this->seq < 0) {
                return;
            }
        }
        else if (t == aodv_msgs::MsgTypes::Rrep)
        {
            rrep.deserialise(payload);
            if ((int)rrep.destSeq - (int)this->seq < 0) {
                return;
            }
        }
        else if (t == aodv_msgs::MsgTypes::Rerr)
        {
            rerr.deserialise(payload);
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
                    row = table.rread(isearch);
                    if (rreq.destSeq > row.destSeq || ((rreq.destSeq > row.destSeq) && (rreq.hopCount+1 < row.hopCount))) { // TODO || sequence number is unknown
                        validSequenceNumber = false; // TODO check if sequence number is invalid
                        if (validSequenceNumber) {
                            /* RFC3561: section 6.4 */
                            eth.ttl = rreq.hopCount + TTL_INCREMENT;
                            while (eth.ttl < TTL_THRESHOLD) {
                                const uint8_t RING_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * (eth.ttl + TIMEOUT_BUFFER);
                                uint16_t timeout = RING_TRAVERSAL_TIME;
                                // TODO wait for RREP
                                eth.ttl += TTL_INCREMENT;
                            }
                            eth.ttl = NET_DIAMETER;
                            uint16_t timeout = NET_TRAVERSAL_TIME;
                            // TODO wait for RREP
                        }
                        // TODO get nextHop
                        uint8_t nextHop;
                        // TODO get precursors from Rreq
                        uint8_t precursors[256];
                        lifetime = ACTIVE_ROUTE_TIMEOUT;
                        table.rupdate(isearch, Route(rreq.destAddr, rreq.destSeq, validSequenceNumber, rreq.hopCount, nextHop, precursors, lifetime));
                    }

                } else { // create route to destination
                    validSequenceNumber = false; // TODO check if sequence number is invalid
                    // TODO get nextHop
                    uint8_t nextHop;
                    // TODO get precursors from Rreq
                    uint8_t precursors[256];
                    lifetime = ACTIVE_ROUTE_TIMEOUT;
                    
                    table.rcreate(Route(rreq.destAddr, rreq.destSeq, validSequenceNumber, rreq.hopCount, nextHop, precursors, lifetime));

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
            if (node has received a RREQ with the same Originator IP Address and RREQ ID within at least the last PATH_DISCOVERY_TIME) {
                silently discard the newly received RREQ.
                return;
            }
            */
            rreq.hopCount = rreq.hopCount + 1; // account for the new hop through the intermediate node
            /* TODO
            row = ...; // searches for a reverse route to the Originator IP Address (see section 6.2), using longest-prefix matching.
            if (If need be) {
                the route is created, or updated using the Originator Sequence Number from the RREQ in its routing table.
            }
            if (rreq.srcAddr > row.destAddr) {
                row.destAddr = req.srcAddr;
            }
            row.validSequenceNumber = true;
            row.nextHop = eth.src;
            row.hopCount = rreq.hopCount;
            minimalLifetime = std::time(0) + 2*NET_TRAVERSAL_TIME + 2*row.hopCount*NODE_TRAVERSAL_TIME;
            row.lifetime = std::max(row.lifetime, minimalLifetime);
            */

            /* RFC3561: section 6.6 */
            // TODO probably need to shift this inside one of the earlier blocks
            bool hasActiveRouteToDestination = false;
            int isearch = table.rsearch(rreq.destAddr);
            if (isearch > 0) { // update route to destination
                hasActiveRouteToDestination = true; // TODO verify if this is enough to be active route
                row = table.rread(isearch);
            }
            if (!(rreq.destAddr == this->addr
                || hasActiveRouteToDestination)) {
                if (eth.ttl > 1) {
                    // update RREQ
                    eth.ttl = eth.ttl - 1;
                    rreq.hopCount = rreq.hopCount + 1; // account for the new hop through the intermediate node
                    // TODO rreq.destSeq = std::max(rreq.destSeq, destination sequence value currently maintained by the node for the requested destination);
                    // the forwarding node MUST NOT modify its maintained value for the destination sequence number, even if the value received in the incoming RREQ is larger than the value currently maintained by the forwarding node.
                    // TODO broadcast RREQ
                }
            } else {
                rrep = aodv_msgs::Rrep();
                rrep.destAddr = rreq.destAddr;
                rrep.srcAddr = rreq.srcAddr;
                // TODO unicast to the next hop toward the originator of the RREQ, as indicated by the route table entry for that originator.

                if (this->addr == rreq.destAddr) { /* RFC3561: section 6.6.1 */
                    if (rreq.destSeq == this->seq + 1) {
                        this->seq = this->seq + 1;
                    }
                    rrep.destSeq = this->seq;
                    rrep.hopCount = 0;
                    rrep.lifetime = MY_ROUTE_TIMEOUT;
                } else { /* RFC3561: section 6.6.2 */
                    // TODO row = forward route entry;
                    rrep.destSeq = row.destSeq;
                    // TODO update forward route entry by placing the last hop node into the precursor list for the forward route entry
                    // TODO update node route entry for the node originating the RREQ by placing the next hop towards the destination in the precursor list for the reverse route entry
                    // TODO rrep.hopCount = distance in hops from the destination;
                    // TODO rrep.lifetime = row.lifetime - std::time(0);
                    if (rreq.flags & aodv_msgs::GRAT_F_MASK) { /* RFC3561: section 6.6.3 */
                        // TODO row = forward route entry;
                        rrep.hopCount = row.hopCount;
                        rrep.destAddr = rreq.srcAddr;
                        rrep.destSeq = rreq.srcSeq;
                        rrep.srcAddr = rreq.destAddr;
                        rrep.lifetime = row.lifetime;
                        // TODO unicast gratuitous RREP to destination node
                    }
                }

                // discard RREQ
            }
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
}
