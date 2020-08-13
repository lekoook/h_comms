#ifndef ETH_H_
#define ETH_H_

#include <stdint.h>
#include <string>
#include <vector>

namespace aodv
{
    /**
     * @brief Byte length of the non-payload part of a Eth.
     * 
     */
    const uint8_t ETH_NONVAR_LEN = 13;

    class Eth
    {
    private:
        /**
         * @brief crc
         *
         */
        uint32_t crc;
  
    public:
        /**
         * @brief sequence number
         * 
         */
        uint32_t seq;
  
        /**
         * @brief dst length
         * 
         */
        uint16_t dstLength;
  
        /**
         * @brief destination address
         * 
         */
        std::string dst;
  
        /**
         * @brief src length
         * 
         */
        uint16_t srcLength;
  
        /**
         * @brief source address
         * 
         */
        std::string src;
  
        /**
         * @brief payload length
         * 
         */
        uint16_t payloadLength;

        /**
         * @brief payload
         *
         */
        std::string payload;

        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Eth();
  
        /**
         * @brief Construct a new Ethernet frame.
         * 
         */
        Eth(uint32_t seq, uint16_t dstLength, std::string dst, uint16_t srcLength, std::string src, uint16_t payloadLength, uint8_t *payload);
  
        /**
         * @brief Deconstructor of the Ethernet frame.
         * 
         */
        ~Eth();

        /**
         * @brief Serializes a object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes.
         */
        void serialise(uint8_t data[]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the object.
         *
         * @param data uint8_t array containing the data to deserialize.
         */
        void deserialise(uint8_t data[]);

        /**
         * @brief Check expected crc with actual crc.
         *
         * @param crc expected crc.
         */
        bool check(uint32_t crc);
    };
}

#endif // ETH_H_
