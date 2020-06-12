#include "Eth.hpp"

namespace aodv
{
    Eth::Eth() : 
        dst(), src(), length()
    {
    }

    Eth::Eth(uint8_t dst, uint8_t src, uint16_t length) :
        dst(dst), src(src), length(length)
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        copyU8(&data[0], dst);
        copyU8(&data[1], src);
        copyU16(&data[2], length);
    }
    
    void Eth::deserialise(uint8_t data[])
    {
        dst = getU8(&data[0]);
        src = getU8(&data[1]);
        length = getU16(&data[2]);
    }
}
