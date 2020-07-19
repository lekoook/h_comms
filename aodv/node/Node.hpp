#ifndef NODE_H_
#define NODE_H_

#include "MsgPeeker.hpp"
#include "../eth/Eth.hpp"
#include "../rerr/Rerr.hpp"
#include "../rrep/Rrep.hpp"
#include "../rrep_ack/RrepAck.hpp"
#include "../rreq/Rreq.hpp"
#include "../table/Table.hpp"
#include "../config.h"
#include <stdint.h>

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
         * @brief precursor list.
         * 
         */
        uint8_t precursors[256];

        /**
         * @brief method that sends data to app level.
         *
         * @param eth ethernet packet.
         */
        void send_app(Eth eth);

        /**
         * @brief method that receives data from app level.
         *
         * @return eth ethernet packet.
         */
        Eth receive_app();

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
        Node(Table table, uint32_t seq, uint32_t id, uint32_t addr, uint8_t precursors[], void (*send_app)(Eth eth), Eth (*receive_app)());
    };
}

#endif // NODE_H_
