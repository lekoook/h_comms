#ifndef MSG_PEEKER_H
#define MSG_PEEKER_H

#include <iostream>
#include <stdint.h>
#include "MsgTypes.hpp"
#include "../utilities/serialisers.hpp"

namespace msg_peeker
{
    aodv_msgs::MsgTypes peekType(uint8_t* msg)
    {
        uint8_t t = serialisers::getU8(msg);
        switch (t)
        {
        case 1:
            return aodv_msgs::MsgTypes::Rreq;
        
        case 2:
            return aodv_msgs::MsgTypes::Rrep;

        case 3:
            return aodv_msgs::MsgTypes::Rerr;

        case 4:
            return aodv_msgs::MsgTypes::RrepAck;

        default:
            std::cerr << "Unknown message type peeked." << std::endl;
            return aodv_msgs::MsgTypes::Unknown; // TODO: What exactly to return here?
        }
    }
}

#endif // MSG_PEEKER_H
