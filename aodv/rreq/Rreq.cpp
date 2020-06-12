#include "Rreq.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv_msgs
{
    Rreq::Rreq() : 
        flags(), hopCount(), id(), 
        destAddr(), destSeq(), srcAddr(), srcSeq()
    {
    }

    Rreq::Rreq(uint8_t flags, uint8_t hopCount, uint32_t id, 
        uint32_t destAddr, uint32_t destSeq, uint32_t srcAddr, uint32_t srcSeq) : 
        flags(flags), hopCount(hopCount), id(id), 
        destAddr(destAddr), destSeq(destSeq), srcAddr(srcAddr), srcSeq(srcSeq)
    {
    }

    void Rreq::serialise(uint8_t data[RREQ_LEN])
    {
        // Write rreq type
        serialisers::copyU8(&data[0], type);

        // Write flags
        uint8_t fTmp = flags << 3; // Shift the 5 lowest sig bits to the 5 most sig bits.
        serialisers::copyU8(&data[1], fTmp);

        // Clean reserved bits.
        serialisers::copyU8(&data[2], 0);

        // Write hop count
        serialisers::copyU8(&data[3], hopCount);

        // Write rreq id
        serialisers::copyU32(&data[4], id);

        // Write dest address
        serialisers::copyU32(&data[8], destAddr);

        // Write dest sequence number
        serialisers::copyU32(&data[12], destSeq);

        // Write src address
        serialisers::copyU32(&data[16], srcAddr);

        // Write src sequence number
        serialisers::copyU32(&data[20], srcSeq);
    }
    
    void Rreq::deserialise(uint8_t data[RREQ_LEN])
    {
        // Get rreq type
        type = serialisers::getU8(&data[0]);

        // Get flags
        uint8_t fTmp = 0;
        fTmp = serialisers::getU8(&data[1]);
        flags = fTmp >> 3; // Shift the 5 most sig bits to 5 lowest sig bits.

        // Get hop count
        hopCount = serialisers::getU8(&data[3]);

        // Get rreq id
        id = serialisers::getU32(&data[4]);

        // Get dest address
        destAddr = serialisers::getU32(&data[8]);

        // Get dest sequence number
        destSeq = serialisers::getU32(&data[12]);

        // Get src address
        srcAddr = serialisers::getU32(&data[16]);

        // Get src sequence number
        srcSeq = serialisers::getU32(&data[20]);
    }
}
