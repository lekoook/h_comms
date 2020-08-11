#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"
#include <iostream>

namespace aodv
{
    Eth::Eth() : 
        seq(), dstLength(), dst(), src(), srcLength(), payloadLength(), payload(), crc()
    {
    }

    Eth::Eth(uint32_t seq, uint16_t dstLength, std::string dst, uint16_t srcLength, std::string src, uint16_t payloadLength, uint8_t *payload) :
        seq(seq), dstLength(dstLength), dst(dst), srcLength(srcLength), src(src), payloadLength(payloadLength), payload(payload), crc()
    {
    }

    Eth::~Eth(){
        std::cout << "before DELETE" << std::endl;
        delete payload;
        std::cout << "after DELETE" << std::endl;
    }

    void Eth::serialise(uint8_t data[])
    {
        std::cout << "serialise" << std::endl;
        size_t i = 0;
        serialisers::copyU32(&data[i], seq);
        i += 4;
        serialisers::copyU16(&data[i], dstLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&dst[0]), dstLength);
        i += dstLength;
        serialisers::copyU16(&data[i], srcLength);
        std::cout << "srcLength: " << srcLength << std::endl;
        printf("%u\n", data[i]);
        printf("%u\n", data[i+1]);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&src[0]), srcLength);
        i += srcLength;
        serialisers::copyU16(&data[i], payloadLength);
        i += 2;
        memcpy(&data[i], payload, payloadLength);
        i += payloadLength;
        serialisers::copyU32(&data[i], crc);
        i += 4;
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        std::cout << "deserialise" << std::endl;
        size_t i = 0;
        seq = serialisers::getU32(&data[i]);
        std::cout << "test-1" << std::endl;
        i += 4;
        dstLength = serialisers::getU16(&data[i]);
        std::cout << "test-2" << std::endl;
        i += 2;
        dst = std::string(dstLength, 1);
        for (int j=0; j<dstLength; j++) {
            dst[j] = data[i+j];
        }        std::cout << "test-3:" << dstLength << std::endl;
        i += dstLength;
        printf("%u\n", data[i]);
        printf("%u\n", data[i+1]);
        srcLength = serialisers::getU16(&data[i]);
        std::cout << "test-4:" << srcLength << std::endl;
        i += 2;
        src = std::string(srcLength, 1);
        for (int j=0; j<srcLength; j++) {
            src[j] = data[i+j];
        }
        i += srcLength;
        payloadLength = serialisers::getU16(&data[i]);
        std::cout << "test-6: " << payloadLength << std::endl;
        i += 2;
        payload = new uint8_t[payloadLength];
        memcpy(payload, &data[i], payloadLength);
        std::cout << "test-7" << std::endl;
        i += payloadLength;
        crc = serialisers::getU32(&data[i]);
        std::cout << "test-8" << std::endl;
        i += 4;
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
