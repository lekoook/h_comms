#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv
{
    Eth::Eth() : 
        ttl(), dst(), src(), length(), payload(), crc()
    {
    }

    Eth::Eth(uint8_t ttl, uint8_t dst, uint8_t src, uint16_t length, uint8_t *payload) :
        ttl(ttl), dst(dst), src(src), length(length), payload(payload), crc()
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        serialisers::copyU8(&data[0], ttl);
        serialisers::copyU8(&data[1], dst);
        serialisers::copyU8(&data[2], src);
        serialisers::copyU16(&data[3], length);
        memcpy(&data[5], payload, length);
        serialisers::copyU32(&data[5+length], crc);
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        ttl = serialisers::getU8(&data[0]);
        dst = serialisers::getU8(&data[1]);
        src = serialisers::getU8(&data[2]);
        length = serialisers::getU16(&data[3]);
        memcpy(payload, &data[5], length);
        crc = serialisers::getU32(&data[5+length]);
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
