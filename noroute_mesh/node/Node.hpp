#ifndef NODE_H_
#define NODE_H_

#include "../eth/Eth.hpp"
#include <stdint.h>
#include <queue>
#include <string>

namespace aodv
{
    /**
     * @brief Address on which broadcasts are made.
     * 
     */
    const uint8_t BROADCAST_ADDR = 0;

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
        uint32_t addr = 0;

        /**
         * @brief FIFO buffer for data packets.
         * 
         */
        std::queue<Eth> fifo;

        /**
         * @brief method that sends data to app level.
         *
         * @param eth ethernet packet.
         */
        void send_app(Eth eth);

        /**
         * @brief method that receives data from app level.
         *
         * @return eth ethernet packet.
         */
        Eth receive_app();

        /**
         * @brief Send payload with this-addr as origin address.
         *
         */
        void originate_payload(uint8_t dst, uint16_t length, uint8_t* payload, void (*send_link)(uint8_t* msg));

        /**
         * @brief Send a control packet or a data packet.
         *
         * @param eth ethernet packet.
         * @param send_link method that sends on link level.
         */
        void send(Eth eth, void (*send_link)(uint8_t* msg));

        /**
         * @brief Receive a control packet or a data packet.
         *
         * @param receive_link method that receives on link level.
         */
        void receive(uint8_t* (*receive_link)(), void (*send_link)(uint8_t* msg));
  
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
        Node(uint32_t seq, uint32_t id, uint32_t addr, void (*send_app)(Eth eth), Eth (*receive_app)());
  
        /**
         * @brief Represent an arbitrary uint8_t buffer as a string.
         * Intersperse a new bit set as 1, every 7 bits in the buffer.
         * Transform first uint8_t with the bits: abcdefgh, into these 9 bits: abcdefg1h.
         * Transform the next uint8_t with the bits: abcdefgh, into these 9 bits: abcdef1gh.
         * Continuing the pattern,
         * abcde1fgh
         * abcd1efgh
         * abc1defgh
         * ab1cdefgh
         * a1bcdefgh
         * 1abcdefg1
         * habcdef1g.
         * 
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
