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
        seq(seq), dstLength(dstLength), dst(dst), srcLength(srcLength), src(src), payloadLength(payloadLength), crc()
    {
    }

    Eth::~Eth(){
        //delete[] payload;
    }

    void Eth::serialise(uint8_t data[])
    {
        size_t i = 0;
        serialisers::copyU32(&data[i], seq);
        i += 4;
        serialisers::copyU16(&data[i], dstLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&dst[0]), dstLength);
        i += dstLength;
        serialisers::copyU16(&data[i], srcLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&src[0]), srcLength);
        i += srcLength;
        serialisers::copyU16(&data[i], payloadLength);
        i += 2;
        //memcpy(&data[i], payload, payloadLength);
        for (int j=0; j<payloadLength; j++) {
            data[i] = payload[j];
            i += 1;
        }
        serialisers::copyU32(&data[i], crc);
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        size_t i = 0;
        seq = serialisers::getU32(&data[i]);
        i += 4;
        dstLength = serialisers::getU16(&data[i]);
        i += 2;
        dst = std::string(dstLength, 1);
        for (int j=0; j<dstLength; j++) {
            dst[j] = data[i+j];
        }
        i += dstLength;
        srcLength = serialisers::getU16(&data[i]);
        i += 2;
        src = std::string(srcLength, 1);
        for (int j=0; j<srcLength; j++) {
            src[j] = data[i+j];
        }
        i += srcLength;
        payloadLength = serialisers::getU16(&data[i]);
        i += 2;
        //payload = new uint8_t[payloadLength];
        //memcpy(payload, &data[i], payloadLength);
        for (int j=0; j<payloadLength; j++) {
            payload.push_back(data[j]);
        }
        i += payloadLength;
        crc = serialisers::getU32(&data[i]);
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc
}
