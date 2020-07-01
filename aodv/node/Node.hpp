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

    /**
     * @brief An array of event.
     */
    typedef struct callbackres_t {
      bool err;
      Eth eth;
    } callbackres_t;

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

        /**
         * @brief Send a control packet or a data packet.
         *
         * @param eth ethernet packet.
         * @param f method that sends on link level.
         */
        void send(Eth eth, void (*f)(uint8_t* msg));

        /**
         * @brief Receive a control packet or a data packet.
         *
         * @param f method that receives on link level.
         */
        void receive(uint8_t* (*f)());
  
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
         * @brief App layer send a data packet to node layer.
         *
         * @param eth ethernet packet.
         * @param cb callback function, app layer receive a data packet from node layer.
         */
        void send_data(Eth eth, callbackres_t (*cb)(Eth eth));
    };
}

#endif // NODE_H_
