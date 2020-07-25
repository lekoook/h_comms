#include "Node.hpp"
#include <stdint.h>

namespace aodv
{
    Node::Node() :
        seq(), id(), addr(), broadcastAddr()
    {
    }

    Node::Node(uint32_t seq, uint32_t id, std::string addr, std::string broadcastAddr) :
        seq(seq), id(id), addr(addr), broadcastAddr(broadcastAddr)
    {
    }

    void Node::send(Eth eth, void (*send_link)(std::string msg, std::string addr))
    {
        // Overwrite seq and src, because this node is originating eth.
        eth.seq = this->seq;
        eth.src = this->addr;
        this->seq++;
        uint16_t length = aodv::ETH_NONPAYLOAD_LEN + eth.payloadLength;
        uint8_t msg[length];
        eth.serialise(msg);
        send_link(this->uint8_to_string(msg, length), this->broadcastAddr);
    }

    void Node::receive(std::string (*receive_link)(), void (*send_link)(std::string msg, std::string addr))
    {
        aodv::Eth eth;

        std::string data = receive_link();
        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        eth.deserialise(msg);

        if (eth.dst == this->addr) {
             fifoToApp.push(eth);
        } else {
            auto search = this->table.find(eth.src);
            if (search == this->table.end()) {
                // Packet does not exist in table.
                this->table[eth.src] = eth.seq;
                this->send(eth, send_link);
            } else {
                if (eth.seq > search->second) {
                    // Packet is newer than table's packet.
                    this->table[search->first] = eth.seq;
                    this->send(eth, send_link);
                }
            }
        }
    }

    std::string Node::uint8_to_string(uint8_t b[], std::string::size_type l)
    {
        /*
         * A uint8_t has bits denoted as: abcdefgh.
         * The next uint8_t may have different bits, but is still denoted as: abcdefgh.
         */
        char c;
        uint8_t s[l*5/4 + l%4 + 1]; // + 1 for '\0'
        std::string::size_type i=0;
        for (; i<l; i+=5) {
            c = 0b10000100u;
            if (i >=l) { b[i] = '\0'; }
            c |= (b[i] >> 4) << 3; // 0b1abcd100
            c |= (b[i] >> 2) & 3; // 0b1abcd1ef
            s[i] = c;
            c = 0b00100001u;
            if (i+1 >=l) { b[i+1] = '\0'; }
            c |= (b[i] & 3) << 6; // 0bgh100001
            c |= (b[i+1] >> 4) << 1; // 0bgh1abcd1
            s[i+1] = c;
            c = 0b00001000u;
            if (i+2 >=l) { b[i+2] = '\0'; }
            c |= (b[i+1] & 15) << 4; // 0befgh1000
            c |= b[i+2] >> 5; // 0befgh1abc
            s[i+2] = c;
            c = 0b01000010u;
            if (i+3 >=l) { b[i+3] = '\0'; }
            c |= ((b[i+2] >> 4) & 1) << 7; // 0bd1000010
            c |= (b[i+2] & 15) << 2; // 0bd1efgh10
            c |= b[i+3] >> 7; // 0bd1efgh1a
            s[i+3] = c;
            c = 0b00010000u;
            if (i+4 >=l) { b[i+4] = '\0'; }
            c |= ((b[i+3] >> 4) & 7) << 5; // 0bbcd10000
            c |= b[i+3] & 15; // 0bbcd1efgh
            s[i+4] = c;
        }
        s[i] = '\0';
        return (char*)s;
    }

    void Node::string_to_uint8(uint8_t b[], std::string s)
    {
        /*
         * Denotation of bits is as in the body of uint8_to_string(uint8_t b[], std::string::size_type l).
         */
        std::string::size_type i=0;
        for (; i<s.size() - 1; i+=4) { // - 1 for '\0'
            /*
             * s[i]   : 0b1abcd1ef
             * s[i+1] : 0bgh1abcd1
             * s[i+2] : 0befgh1abc
             * s[i+3] : 0bd1efgh1a
             * s[i+4] : 0bbcd1efgh
             */
            b[i] = ((s[i] & 0b01111000u) << 1) | ((s[i] & 0b00000011u) << 2) | ((s[i+1] & 0b11000000u) >> 6);
            b[i+1] = ((s[i+1] & 0b00011110u) << 3) | ((s[i+2] & 0b11110000u) >> 4);
            b[i+2] = ((s[i+2] & 0b00000111u) << 5) | ((s[i+3] & 0b10000000u) >> 3) | ((s[i+3] & 0b00111100u) >> 2);
            b[i+3] = ((s[i+3] & 0b00000001u) << 7) | ((s[i+4] & 0b11100000u) >> 1) | (s[i+4] & 0b00001111u);
        }
    }
}
