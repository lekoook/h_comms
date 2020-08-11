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
        eth.serialise(msg);
        std::string s = uint8_to_string(msg, length);

        for (int i = 0; i < length; i++)
        {
            printf("%u ", (uint8_t)msg[i]);
        }
        printf("\n\n");
        for (int i = 0; i < s.length(); i++)
        {
            printf("%u ", (uint8_t)s[i]);
        }
        printf("\n\n");
        printf("Before SendTo\n");
        //commsClient->SendTo(this->uint8_to_string(msg, length), this->broadcastAddr);
        //std::string str = "hello";
        commsClient->SendTo(s, this->broadcastAddr);
        printf("After SendTo\n");
    }

    tl::optional<aodv::Eth> Node::receive(std::string data, subt::CommsClient* commsClient)
    {
        aodv::Eth eth;

        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        for (int i = 0; i < data.length(); i++)
        {
            printf("%u ", (uint8_t)data[i]);
        }
        printf("\n\n");
        for (int i = 0; i < data.length(); i++)
        {
            printf("%u ", (uint8_t)msg[i]);
        }
        printf("\n\n");
        eth.deserialise(msg);

        std::cout << "Ethernet Contents:" << std::endl
            << "seq:" << eth.seq << std::endl
            << "dstlength:" << eth.dstLength << std::endl
            << "dst:" << eth.dst << std::endl
            << "srclength:" << eth.srcLength<< std::endl
            << "src:" << eth.src << std::endl;

        if (eth.dst == this->addr) {
            return eth; // return optional object that contains eth.
            std::cout << "Package for me" << std::endl;
        } else {
            auto search = this->table.find(eth.src);
            if (search == this->table.end()) {
                // Packet does not exist in table.
                this->table[eth.src] = eth.seq;
                this->send(eth, commsClient, false);
                std::cout << "testA" << std::endl;
            } else {
                if (eth.seq > search->second) {
                    // Packet is newer than table's packet.
                    this->table[search->first] = eth.seq;
                    this->send(eth, commsClient, false);
                    std::cout << "testB" << std::endl;
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
