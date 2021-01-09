#ifndef H_PACKET
#define H_PACKET

#include <cstring>
#include <string>
#include <vector>
#include "serialisers.hpp"

namespace ptp_comms
{

/**
 * @brief Represents a single packet that can be transmitted or received.
 * 
 */
class Packet
{
public:
    /**
     * @brief Total length of fixed-length fields.
     * @details isAck(1) + seqNum(4) + segNum(4) + totalLength(8)
     * 
     */
    static uint8_t const FIXED_LEN = 17;

    /**
     * @brief Total allowable size (bytes) of a Packet.
     * 
     */
    static uint32_t const MAX_PACKET_SIZE = 1500; // Constrained by subt API

    /**
     * @brief Total allowable size (bytes) of a segment. MAX_SEGMENT_SIZE = MAX_PACKET_SIZE - FIXED_LEN
     * 
     */
    static uint32_t const MAX_SEGMENT_SIZE = MAX_PACKET_SIZE - FIXED_LEN;

    /**
     * @brief Packet field that indicate if this is an acknowledgment packet.
     * 
     */
    bool isAck = false;

    /**
     * @brief Packet field that indicate sequence number of this packet.
     * 
     */
    uint32_t seqNum = 0;

    /**
     * @brief Packet field that indicate the index number of this segment.
     * 
     */
    uint32_t segNum = 0;

    /**
     * @brief Packet field that indicate the total length of the full data.
     * 
     */
    uint64_t totalLength = 0;

    /**
     * @brief Packet field containing the actual payload data.
     * 
     */
    std::vector<uint8_t> data;

    /**
     * @brief Construct a new Packet object.
     * 
     */
    Packet() {}

    /**
     * @brief Construct a new Packet object.
     * 
     * @param seqNum Sequence number of this packet.
     * @param segNum Segment number of this packet.
     * @param totalLength Total length of full data.
     * @param data Payload data.
     * @param isAck True if this is an acknowledgement packet, false otherwise.
     */
    Packet(uint32_t seqNum, uint32_t segNum, uint64_t totalLength, std::vector<uint8_t> data, bool isAck=false)
        : seqNum(seqNum), segNum(segNum), totalLength(totalLength), data(data), isAck(isAck)
    {}

    /**
     * @brief Serializes the Packet into a std::string.
     * 
     * @return std::string Serialized string.
     */
    std::string serialize()
    {
        uint32_t tLen = FIXED_LEN + data.size();
        uint8_t temp[tLen] = { 0 };

        serialisers::copyU8(&temp[0], isAck ? 1 : 0);
        serialisers::copyU32(&temp[1], seqNum);
        serialisers::copyU64(&temp[5], totalLength);
        serialisers::copyU32(&temp[13], segNum);
        memcpy(&temp[17], data.data(), data.size());

        return std::string((char*)temp, tLen);
    }

    /**
     * @brief Deserializes a std::string into a Packet. Caller must ensure length of string is correct.
     * 
     * @param byteStr std::string to deserialize from.
     */
    void deserialize(std::string byteStr)
    {
        uint8_t* temp = (uint8_t*) byteStr.data();
        isAck = (serialisers::getU8(&temp[0]) == 1);
        seqNum = serialisers::getU32(&temp[1]);
        totalLength = serialisers::getU64(&temp[5]);
        segNum = serialisers::getU32(&temp[13]);
        uint64_t dataLen = byteStr.size() - FIXED_LEN;
        uint8_t* dStart = &temp[17];
        data = std::vector<uint8_t>(dStart, dStart + dataLen);
    }
};

}

#endif // H_PACKET