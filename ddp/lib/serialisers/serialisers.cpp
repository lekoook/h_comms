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
        val |= (src[0] & 0xff) << 8;
        val |= (src[1] & 0xff);
        return val;
    }

    uint32_t getU32(uint8_t* src)
    {
        uint32_t val = 0;
        val |= (src[0] & 0xff) << 24;
        val |= (src[1] & 0xff) << 16;
        val |= (src[2] & 0xff) << 8;
        val |= (src[3] & 0xff);
        return val;
    }

    uint64_t getU64(uint8_t* src)
    {
        uint64_t val = 0;
        val |= (src[0] & 0xff) << 56;
        val |= (src[1] & 0xff) << 48;
        val |= (src[2] & 0xff) << 40;
        val |= (src[3] & 0xff) << 32;
        val |= (src[4] & 0xff) << 24;
        val |= (src[5] & 0xff) << 16;
        val |= (src[6] & 0xff) << 8;
        val |= (src[7] & 0xff);
        return val;
    }
}
