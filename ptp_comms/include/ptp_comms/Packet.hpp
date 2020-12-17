#ifndef H_PACKET
#define H_PACKET

#include <cstring>
#include <string>
#include <vector>
#include "serialisers.hpp"

class Packet
{
public:
    static uint8_t const FIXED_LEN = 7;

    bool isAck = false;
    uint32_t seqNum = 0;
    uint8_t totalSegs = 0;
    uint8_t segNum = 0;
    std::vector<uint8_t> data;

    Packet() {}

    Packet(uint32_t seqNum, uint8_t totalSegs, uint8_t segNum, std::vector<uint8_t> data, bool isAck=false)
        : seqNum(seqNum), totalSegs(totalSegs), segNum(segNum), data(data), isAck(isAck)
    {}

    std::string serialize()
    {
        uint32_t tLen = FIXED_LEN + data.size();
        uint8_t temp[tLen] = { 0 };

        serialisers::copyU8(&temp[0], isAck ? 1 : 0);
        serialisers::copyU32(&temp[1], seqNum);
        serialisers::copyU8(&temp[5], totalSegs);
        serialisers::copyU8(&temp[6], segNum);
        memcpy(&temp[7], data.data(), data.size());

        return std::string((char*)temp, tLen);
    }

    void deserialize(std::string byteStr)
    {
        uint8_t* temp = (uint8_t*) byteStr.data();
        isAck = (serialisers::getU8(&temp[0]) == 1);
        seqNum = serialisers::getU32(&temp[1]);
        totalSegs = serialisers::getU8(&temp[5]);
        segNum = serialisers::getU8(&temp[6]);
        uint32_t dataLen = byteStr.size() - FIXED_LEN;
        uint8_t* dStart = &temp[7];
        data = std::vector<uint8_t>(dStart, dStart + dataLen);
    }
};

#endif // H_PACKET