#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv
{
    Eth::Eth() : 
        seq(), dstLength(), dst(), src(), srcLength(), payloadLength(), payload(), crc()
    {
    }

    Eth::Eth(uint32_t seq, uint16_t dstLength, std::string dst, uint16_t srcLength, std::string src, uint16_t payloadLength, uint8_t *payload) :
        seq(seq), dstLength(dstLength), dst(dst), srcLength(srcLength), src(src), payloadLength(payloadLength), payload(payload), crc()
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        size_t i = 0;
        serialisers::copyU32(&data[i], seq);
        i += 4;
        serialisers::copyU16(&data[i], dstLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&dst[0]), dstLength);
        i += dstLength;
        serialisers::copyU16(&data[i], srcLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&src[0]), srcLength);
        i += srcLength;
        serialisers::copyU16(&data[i], payloadLength);
        i += 2;
        memcpy(&data[i], payload, payloadLength);
        i += payloadLength;
        serialisers::copyU32(&data[i], crc);
        i += 4;
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        size_t i = 0;
        seq = serialisers::getU32(&data[i]);
        i += 4;
        dstLength = serialisers::getU16(&data[i]);
        i += 2;
        memcpy(reinterpret_cast<uint8_t*>(&dst[0]), &data[i], dstLength);
        i += dstLength;
        srcLength = serialisers::getU16(&data[i]);
        i += 2;
        memcpy(reinterpret_cast<uint8_t*>(&src[0]), &data[i], dstLength);
        i += srcLength;
        payloadLength = serialisers::getU16(&data[i]);
        i += 2;
        memcpy(payload, &data[i], payloadLength);
        i += payloadLength;
        crc = serialisers::getU32(&data[i]);
        i += 4;
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
