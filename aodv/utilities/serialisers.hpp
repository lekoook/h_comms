#ifndef SERIALISERS_H
#define SERIALISERS_H

#include <stdint.h>

/**
 * @brief This namespace contains the functionalities to serialize object and values into bytes array.
 * 
 */
namespace serialisers
{
    /**
     * @brief Copies a uint8_t value into a uint8_t bytes array.
     * 
     * @param dest uint8_t array to store the copied bytes. MUST be at least 1 byte length.
     * @param src uint8_t value to copy.
     */
    void copyU8(uint8_t* dest, uint8_t src)
    {
        dest[0] = src;
    }

    /**
     * @brief Copies a uint32_t value into a uint8_t bytes array.
     * 
     * @param dest uint8_t array to store the copied bytes. MUST be at least 4 bytes length.
     * @param src uint32_t value to copy.
     */
    void copyU32(uint8_t* dest, uint32_t src)
    {
        dest[0] = (uint8_t) ((src >> 24) & 0xff);
        dest[1] = (uint8_t) ((src >> 16) & 0xff);
        dest[2] = (uint8_t) ((src >> 8) & 0xff);
        dest[3] = (uint8_t) (src & 0xff);
    }

    /**
     * @brief Retrieves a uint8_t value from a uint8_t bytes array.
     * 
     * @param src uint8_t array to retrieve from. MUST be at least 1 byte length.
     * @return uint8_t value retrieved.
     */
    uint8_t getU8(uint8_t* src)
    {
        return src[0];
    }

    /**
     * @brief Retrieves a uint32_t value from a uint8_t bytes array.
     * 
     * @param src uint8_t array to retrieve from. MUST be at least 4 bytes length.
     * @return uint32_t value retrieved.
     */
    uint32_t getU32(uint8_t* src)
    {
        uint32_t val = 0;
        val |= (src[0] & 0xff) << 24;
        val |= (src[1] & 0xff) << 16;
        val |= (src[2] & 0xff) << 8;
        val |= (src[3] & 0xff);
        return val;
    }
}

#endif // SERIALISERS_H