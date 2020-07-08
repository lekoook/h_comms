#ifndef MSG_PEEKER_H
#define MSG_PEEKER_H

#include "MsgTypes.hpp"
#include "../utilities/serialisers.hpp"

namespace msg_peeker
{
    aodv_msgs::MsgTypes peekType(uint8_t* msg);
}

#endif // MSG_PEEKER_H
