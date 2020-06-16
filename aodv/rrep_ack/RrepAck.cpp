#include "RrepAck.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv_msgs
{
    RrepAck::RrepAck(){}

    void RrepAck::serialise(uint8_t data[RREP_ACK_LEN])
    {
        // Write rreq type
        serialisers::copyU8(&data[0], type);

        // Write clean reserved bits
        serialisers::copyU8(&data[1], 0);
    }
    
    void RrepAck::deserialise(uint8_t data[RREP_ACK_LEN])
    {
        // Get rreq type
        type = serialisers::getU8(&data[0]);
    }
}
