#include "Rreq.hpp"

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
        copyU8(&data[0], type);

        // Write flags
        uint8_t fTmp = flags << 3; // Shift the 5 lowest sig bits to the 5 most sig bits.
        copyU8(&data[1], fTmp);

        // Clean reserved bits.
        copyU8(&data[2], 0);

        // Write hop count
        copyU8(&data[3], hopCount);

        // Write rreq id
        copyU32(&data[4], id);

        // Write dest address
        copyU32(&data[8], destAddr);

        // Write dest sequence number
        copyU32(&data[12], destSeq);

        // Write src address
        copyU32(&data[16], srcAddr);

        // Write src sequence number
        copyU32(&data[20], srcSeq);
    }
    
    void Rreq::deserialise(uint8_t data[RREQ_LEN])
    {
        // Get rreq type
        type = getU8(&data[0]);

        // Get flags
        uint8_t fTmp = 0;
        fTmp = getU8(&data[1]);
        flags = fTmp >> 3; // Shift the 5 most sig bits to 5 lowest sig bits.

        // Get hop count
        hopCount = getU8(&data[3]);

        // Get rreq id
        id = getU32(&data[4]);

        // Get dest address
        destAddr = getU32(&data[8]);

        // Get dest sequence number
        destSeq = getU32(&data[12]);

        // Get src address
        srcAddr = getU32(&data[16]);

        // Get src sequence number
        srcSeq = getU32(&data[20]);
    }
    
    /**
     * @brief Bytes are bit shifted instead of using memcpy to be more portable.
     * 
     */
    void Rreq::copyU8(uint8_t* dest, uint8_t src)
    {
        dest[0] = (uint8_t) ((src >> 0) & 0xff);
    }

    /**
     * @brief Bytes are bit shifted instead of using memcpy to be more portable.
     * 
     */
    void Rreq::copyU32(uint8_t* dest, uint32_t src)
    {
        dest[0] = (uint8_t) ((src >> 24) & 0xff);
        dest[1] = (uint8_t) ((src >> 16) & 0xff);
        dest[2] = (uint8_t) ((src >> 8) & 0xff);
        dest[3] = (uint8_t) ((src >> 0) & 0xff);
    }
    
    /**
     * @brief Bytes are bit shifted instead of using memcpy to be more portable.
     * 
     */
    uint8_t Rreq::getU8(uint8_t* src)
    {
        return src[0];
    }
    
    /**
     * @brief Bytes are bit shifted instead of using memcpy to be more portable.
     * 
     */
    uint32_t Rreq::getU32(uint8_t* src)
    {
        uint32_t val = 0;
        val |= (src[0] & 0xff) << 24;
        val |= (src[1] & 0xff) << 16;
        val |= (src[2] & 0xff) << 8;
        val |= (src[3] & 0xff) << 0;
        return val;
    }
}
