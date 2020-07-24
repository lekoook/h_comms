#ifndef NODE_H_
#define NODE_H_

#include "../eth/Eth.hpp"
#include <stdint.h>
#include <queue>
#include <string>
#include <unordered_map>

namespace aodv
{
    class Node
    {
    private:
        /**
         * @brief Sequence number.
         * 
         */
        uint32_t seq = 0;
        
        /**
         * @brief RREQ identifier.
         * 
         */
        uint32_t id = 0;

        /**
         * @brief address.
         * 
         */
        std::string addr;

        /**
         * @brief FIFO buffer for data packets from app to node.
         * 
         */
        std::queue<Eth> fifoFromApp;

        /**
         * @brief FIFO buffer for data packets from node to app.
         * 
         */
        std::queue<Eth> fifoToApp;

        /**
        * @brief Address on which broadcasts are made.
        * 
        */
        std::string broadcastAddr;

        /**
        * @brief Address on which broadcasts are made.
        * 
        */
        std::unordered_map<std::string, uint32_t> table;

        /**
         * @brief Receive a control packet or a data packet.
         *
         * @param receive_link method that receives on link level.
         */
        void receive(std::string (*receive_link)(), void (*send_link)(std::string msg, std::string addr));
  
    public:
        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Node();
  
        /**
         * @brief Construct a new object.
         * 
         */
        Node(uint32_t seq, uint32_t id, std::string addr, std::string broadcastAddr);

        /**
         * @brief Send a control packet or a data packet with this->seq and this->addr.
         *
         * @param eth ethernet packet.
         * @param send_link method that sends on link level.
         */
        void send(Eth eth, void (*send_link)(std::string msg, std::string addr));
  
        /**
         * @brief Represent an arbitrary uint8_t buffer as a string.
         * Transform each uint8_t with the bits: abcdefgh, into these 10 bits: 1abcd1efgh.
         * Concatenate all bits from all uint8_t, then add a terminating '\0'.
         * 
         * @param b uint8_t buffer.
         * @param l length of uint8_t buffer.
         * @return string representation.
         */
        std::string uint8_to_string(uint8_t b[], std::string::size_type l);
  
        /**
         * @brief Parse represented string as an arbitrary uint8_t buffer.
         * Inverse of uint8_to_string(uint8_t b[], std::string::size_type l).
         * 
         * @param b uint8_t buffer.
         * @param s string.
         */
        void string_to_uint8(uint8_t b[], std::string s);
    };
}

#endif // NODE_H_
