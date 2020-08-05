#include "Node.hpp"
#include <stdint.h>

namespace aodv
{
    Node::Node() :
        id(), addr(), broadcastAddr()
    {
    }

    Node::Node(uint32_t id, std::string addr, std::string broadcastAddr) :
        id(id), addr(addr), broadcastAddr(broadcastAddr)
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

    tl::optional<aodv::Eth> Node::receive(std::string data, void (*send_link)(std::string msg, std::string addr))
    {
        aodv::Eth eth;

        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        eth.deserialise(msg);

        if (eth.dst == this->addr) {
             return eth; // return optional object that contains eth.
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

        return tl::nullopt; // return optional object that does not contain a value.
    }

    std::string Node::uint8_to_string(uint8_t b[], std::string::size_type l)
    {
        /*
         * A uint8_t has bits denoted as: abcdefgh.
         */
        char c;
        uint8_t s[l*2 + 1]; // + 1 for '\0'
        std::string::size_type i=0;
        for (; i<l; i++) {
            c = 0b10101010u;
            c |= (b[i] >> 7) << 6;
            c |= ((b[i] >> 6) & 1) << 4;
            c |= ((b[i] >> 5) & 1) << 2;
            c |= (b[i] >> 4) & 1;
            s[2*i] = c;
            c = 0b10101010u;
            c |= ((b[i] >> 3) & 1) << 6;
            c |= ((b[i] >> 2) & 1) << 4;
            c |= ((b[i] >> 1) & 1) << 2;
            c |= b[i] & 1;
            s[2*i+1] = c;
        }
        s[2*i] = '\0';
        return (char*)s;
    }

    void Node::string_to_uint8(uint8_t b[], std::string s)
    {
        /*
         * Denotation of bits is as in the body of uint8_to_string(uint8_t b[], std::string::size_type l).
         */
        for (std::string::size_type i=0; i<s.size(); i+=2) {
            b[i/2] = ((s[i] & 0b01000000u) << 1) | ((s[i] & 0b00010000u) << 2) | ((s[i] & 0b00000100u) << 3) | ((s[i] & 1) << 4);
            b[i/2] |= ((s[i+1] & 0b01000000u) >> 3) | ((s[i+1] & 0b00010000u) >> 2) | ((s[i+1] & 0b00000100u) >> 1) | (s[i+1] & 1);
        }
    }
}
