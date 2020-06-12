#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv
{
    Eth::Eth() : 
        dst(), src(), length(), payload(), crc()
    {
    }

    Eth::Eth(uint8_t dst, uint8_t src, uint16_t length, uint8_t *payload) :
        dst(dst), src(src), length(length), payload(payload), crc()
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        serialisers::copyU8(&data[0], dst);
        serialisers::copyU8(&data[1], src);
        serialisers::copyU16(&data[2], length);
        memcpy(&data[4], payload, length);
        serialisers::copyU32(&data[4+length], crc);
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        dst = serialisers::getU8(&data[0]);
        src = serialisers::getU8(&data[1]);
        length = serialisers::getU16(&data[2]);
        memcpy(payload, &data[4], length);
        crc = serialisers::getU32(&data[4+length]);
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
