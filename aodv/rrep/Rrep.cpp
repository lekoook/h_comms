#include "Rrep.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv_msgs
{
    Rrep::Rrep() : 
        flags(), prefixSz(), hopCount(), 
        destAddr(), destSeq(), srcAddr(), lifetime()
    {
    }

    Rrep::Rrep(uint8_t flags, uint8_t prefixSz, uint8_t hopCount, 
        uint32_t destAddr, uint32_t destSeq, uint32_t srcAddr, uint32_t lifetime) : 
        flags(flags), prefixSz(prefixSz), hopCount(hopCount), 
        destAddr(destAddr), destSeq(destSeq), srcAddr(srcAddr), lifetime(lifetime)
    {
    }

    void Rrep::serialise(uint8_t data[RREP_LEN])
    {
        // Write rreq type
        serialisers::copyU8(&data[0], type);

        // Write flags
        uint8_t fTmp = flags << 6; // Shift the 5 lowest sig bits to the 5 most sig bits.
        serialisers::copyU8(&data[1], fTmp);

        // Write prefix size
        serialisers::copyU8(&data[2], prefixSz);

        // Write hop count
        serialisers::copyU8(&data[3], hopCount);

        // Write dest address
        serialisers::copyU32(&data[4], destAddr);

        // Write dest sequence number
        serialisers::copyU32(&data[8], destSeq);

        // Write src address
        serialisers::copyU32(&data[12], srcAddr);

        // Write src sequence number
        serialisers::copyU32(&data[16], lifetime);
    }
    
    void Rrep::deserialise(uint8_t data[RREP_LEN])
    {
        // Get rreq type
        type = serialisers::getU8(&data[0]);

        // Get flags
        uint8_t fTmp = 0;
        fTmp = serialisers::getU8(&data[1]);
        flags = fTmp >> 6; // Shift the 5 most sig bits to 5 lowest sig bits.

        // Get prefix size
        prefixSz = serialisers::getU8(&data[2]);

        // Get hop count
        hopCount = serialisers::getU8(&data[3]);

        // Get dest address
        destAddr = serialisers::getU32(&data[4]);

        // Get dest sequence number
        destSeq = serialisers::getU32(&data[8]);

        // Get src address
        srcAddr = serialisers::getU32(&data[12]);

        // Get lifetime
        lifetime = serialisers::getU32(&data[16]);
    }
}
