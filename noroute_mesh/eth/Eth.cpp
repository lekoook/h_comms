#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv
{
    Eth::Eth() : 
        seq(), dst(), src(), length(), payload(), crc()
    {
    }

    Eth::Eth(uint32_t seq, uint8_t dst, uint8_t src, uint16_t length, uint8_t *payload) :
        seq(seq), dst(dst), src(src), length(length), payload(payload), crc()
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        serialisers::copyU32(&data[0], seq);
        serialisers::copyU8(&data[4], dst);
        serialisers::copyU8(&data[5], src);
        serialisers::copyU16(&data[6], length);
        memcpy(&data[8], payload, length);
        serialisers::copyU32(&data[8+length], crc);
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        seq = serialisers::getU32(&data[0]);
        dst = serialisers::getU8(&data[4]);
        src = serialisers::getU8(&data[5]);
        length = serialisers::getU16(&data[6]);
        memcpy(payload, &data[8], length);
        crc = serialisers::getU32(&data[8+length]);
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
