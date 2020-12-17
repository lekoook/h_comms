#ifndef H_PACKET
#define H_PACKET

#include <cstring>
#include <string>
#include <vector>
#include "serialisers.hpp"

class Packet
{
private:
    uint8_t const FIXED_LEN = 9;

public:
    bool isAck = false;
    uint32_t seqNum = 0;
    uint8_t totalSegs = 0;
    uint8_t segNum = 0;
    uint8_t srcLen = 0;
    std::string src;
    uint8_t destLen = 0;
    std::string dest;
    std::vector<uint8_t> data;

    Packet(uint32_t seqNum, uint8_t totalSegs, uint8_t segNum, std::string src, std::string dest, std::vector<uint8_t> data, bool isAck=false)
        : seqNum(seqNum), totalSegs(totalSegs), segNum(segNum), src(src), dest(dest), data(data), isAck(isAck)
    {
        srcLen = src.size();
        destLen = dest.size();
    }

    std::string serialize()
    {
        uint32_t tLen = FIXED_LEN + srcLen + destLen + data.size();
        uint8_t temp[tLen] = { 0 };

        serialisers::copyU8(&temp[0], isAck ? 1 : 0);
        serialisers::copyU32(&temp[1], seqNum);
        serialisers::copyU8(&temp[5], totalSegs);
        serialisers::copyU8(&temp[6], segNum);
        uint8_t cLen = (srcLen << 4) | (destLen & 0x0F);
        serialisers::copyU8(&temp[7], cLen);
        memcpy(&temp[8], src.data(), srcLen);
        memcpy(&temp[8+srcLen], dest.data(), destLen);
        memcpy(&temp[8+srcLen+destLen], data.data(), data.size());
    }

    void deserialize(std::string byteStr)
    {
        uint8_t* temp = (uint8_t*) byteStr.data();
        isAck = (serialisers::getU8(&temp[0]) == 1);
        seqNum = serialisers::getU32(&temp[1]);
        totalSegs = serialisers::getU8(&temp[5]);
        segNum = serialisers::getU8(&temp[6]);
        uint8_t cLen = serialisers::getU8(&temp[7]);
        srcLen = cLen >> 4;
        destLen = cLen & 0x0F;
        src = std::string((char*)(&temp[8]), srcLen);
        dest = std::string((char*)(&temp[8+srcLen]), destLen);
        uint32_t dataLen = byteStr.size() - FIXED_LEN - srcLen - destLen;
        uint8_t* dStart = &temp[8+srcLen+destLen];
        data = std::vector<uint8_t>(dStart, dStart + dataLen);
    }
};

#endif // H_PACKET