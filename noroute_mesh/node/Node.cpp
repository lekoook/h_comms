#include "Node.hpp"
#include <stdint.h>
#include <iostream>
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

    void Node::send(Eth &eth, subt::CommsClient* commsClient, bool isOriginating)
    {
        if (isOriginating) {
            // Overwrite seq and src, because this node is originating eth.
            eth.seq = this->seq;
            eth.src = this->addr;
            this->seq++;
        }

        uint16_t length = aodv::ETH_NONVAR_LEN + eth.srcLength + eth.dstLength + eth.payloadLength;
        uint8_t msg[length];

        std::cout << "SENDING" << std::endl
            << eth.seq << std::endl
            << eth.dst << std::endl
            << eth.dstLength << std::endl
            << eth.src << std::endl
            << eth.srcLength << std::endl
            << eth.payloadLength << std::endl;
        eth.serialise(msg);
        std::string s = uint8_to_string(msg, length);
        commsClient->SendTo(s, this->broadcastAddr);
    }

    tl::optional<aodv::Eth> Node::receive(std::string data, subt::CommsClient* commsClient)
    {
        aodv::Eth eth;

        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        eth.deserialise(msg);

        std::cout << "RECVING" << std::endl
            << eth.seq << std::endl
            << eth.dst << std::endl
            << eth.dstLength << std::endl
            << eth.src << std::endl
            << eth.srcLength << std::endl
            << eth.payloadLength << std::endl;

        if (eth.src == this->addr) {
            std::cout << "Ignore packet from myself" << std::endl;
            return tl::nullopt;
        }
        
        if (eth.dst == this->addr) {
            std::cout << "Packet for me" << std::endl;
            return eth; // return optional object that contains eth.
        }
        // } else {
        //     auto search = this->table.find(eth.src);
        //     if (search == this->table.end()) {
        //         // Packet does not exist in table.
        //         this->table[eth.src] = eth.seq;
        //         this->send(eth, commsClient, false);
        //         std::cout << "Packet not in table" << std::endl;
        //     } else {
        //         if (eth.seq > search->second) {
        //             // Packet is newer than table's packet.
        //             this->table[search->first] = eth.seq;
        //             this->send(eth, commsClient, false);
        //             std::cout << "Packet in table" << std::endl;
        //         }
        //     }
        // }

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
        return std::string((char*)s,l*2+1);
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
