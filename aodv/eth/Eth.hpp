#ifndef ETH_H_
#define ETH_H_

#include <stdint.h>

namespace aodv
{
    class Eth
    {
    private:
  
    public:
        /**
         * @brief destination address
         * 
         */
        uint8_t dst;
  
        /**
         * @brief source address
         * 
         */
        uint8_t src;
  
        /**
         * @brief payload length
         * 
         */
        uint16_t length;

        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Eth();
  
        /**
         * @brief Construct a new Ethernet frame.
         * 
         */
        Eth(uint8_t dst, uint8_t src, uint16_t length);
  
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
    };
}

#endif // ETH_H_
