#include "Eth.hpp"

namespace aodv
{
    Eth::Eth() : 
        dst(), src(), length(), payload(), crc()
    {
    }

    Eth::Eth(uint8_t dst, uint8_t src, uint16_t length, uint8_t *payload, uint32_t crc) :
        dst(dst), src(src), length(length), payload(payload), crc(crc)
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        copyU8(&data[0], dst);
        copyU8(&data[1], src);
        copyU16(&data[2], length);
        memcpy(&data[4], payload, length);
        copyU32(&data[4+length], crc);
    }
    
    void Eth::deserialise(uint8_t data[])
    {
        dst = getU8(&data[0]);
        src = getU8(&data[1]);
        length = getU16(&data[2]);
        memcpy(payload, &data[4], length);
        crc = getU32(&data[4+length]);
    }
}
