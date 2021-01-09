#include "serialisers.hpp"

namespace serialisers
{
    void copyU8(uint8_t* dest, uint8_t src)
    {
        dest[0] = src;
    }

    void copyU16(uint8_t* dest, uint16_t src)
    {
        dest[0] = (uint8_t) ((src >> 8) & 0xff);
        dest[1] = (uint8_t) (src & 0xff);
    }

    void copyU32(uint8_t* dest, uint32_t src)
    {
        dest[0] = (uint8_t) ((src >> 24) & 0xff);
        dest[1] = (uint8_t) ((src >> 16) & 0xff);
        dest[2] = (uint8_t) ((src >> 8) & 0xff);
        dest[3] = (uint8_t) (src & 0xff);
    }

    void copyU64(uint8_t* dest, uint64_t src)
    {
        dest[0] = (uint8_t) ((src >> 56) & 0xff);
        dest[1] = (uint8_t) ((src >> 48) & 0xff);
        dest[2] = (uint8_t) ((src >> 40) & 0xff);
        dest[3] = (uint8_t) ((src >> 32) & 0xff);
        dest[4] = (uint8_t) ((src >> 24) & 0xff);
        dest[5] = (uint8_t) ((src >> 16) & 0xff);
        dest[6] = (uint8_t) ((src >> 8) & 0xff);
        dest[7] = (uint8_t) (src & 0xff);
    }

    uint8_t getU8(uint8_t* src)
    {
        return src[0];
    }

    uint16_t getU16(uint8_t* src)
    {
        uint16_t val = 0;
        val |= (uint16_t)src[0] << 8;
        val |= (uint16_t)src[1];
        return val;
    }

    uint32_t getU32(uint8_t* src)
    {
        uint32_t val = 0;
        val |= (uint32_t)src[0] << 24;
        val |= (uint32_t)src[1] << 16;
        val |= (uint32_t)src[2] << 8;
        val |= (uint32_t)src[3];
        return val;
    }

    uint64_t getU64(uint8_t* src)
    {
        uint64_t val = 0;
        val |= (uint64_t)src[0] << 56;
        val |= (uint64_t)src[1] << 48;
        val |= (uint64_t)src[2] << 40;
        val |= (uint64_t)src[3] << 32;
        val |= (uint64_t)src[4] << 24;
        val |= (uint64_t)src[5] << 16;
        val |= (uint64_t)src[6] << 8;
        val |= (uint64_t)src[7];
        return val;
    }
}
