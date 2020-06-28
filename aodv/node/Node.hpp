#ifndef NODE_H_
#define NODE_H_

#include "MsgPeeker.hpp"
#include "../eth/Eth.hpp"
#include "../rrep/Rrep.hpp"
#include "../rrep_ack/RrepAck.hpp"
#include "../rreq/Rreq.hpp"
#include "../table/Table.hpp"
#include "../config.h"
#include <stdint.h>
#include <stack>

namespace aodv
{
    class Node
    {
    private:
        /**
         * @brief Routing table object.
         * 
         */
        Table table;
  
        /**
         * @brief Sequence number.
         * 
         */
        uint32_t seq = 0;
        
        /**
         * @brief RREQ identifier.
         * 
         */
        uint32_t id = 0;

        /**
         * @brief address.
         * 
         */
        uint32_t addr = 0;
  
        /**
         * @brief processing stack.
         * // RFC3561: section 6.5
         * 
         */
        std::stack<rreqStack_t> rreqStack;
  
    public:
        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Node();
  
        /**
         * @brief Construct a new object.
         * 
         */
        Node(Table table, uint32_t seq, uint32_t id, uint32_t addr);

        /**
         * @brief Send a control packet or a data packet.
         *
         * @param eth ethernet packet.
         */
        void send(Eth eth);

        /**
         * @brief Receive a control packet or a data packet.
         *
         * @param msg packet
         */
        void receive(uint8_t* msg);
    };
}

#endif // NODE_H_
