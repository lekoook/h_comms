#ifndef H_DATA_MSG
#define H_DATA_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"
#include "serialisers.hpp"

/**
 * @brief Represents the data message for data distribution.
 * 
 */
class DataMsg : public BaseMsg
{
private:
    /**
     * @brief Total length of fixed-length fields.
     * @details Length = MsgType(1) + RequestSequence(4) + EntryID(2) + Timestamp(8)
     * 
     */
    const uint8_t FIXED_LEN = 15;

public:
    /**
     * @brief Request sequence number this data is for.
     * 
     */
    uint32_t reqSequence;

    /**
     * @brief The entry ID of the data.
     * 
     */
    uint16_t entryId;

    /**
     * @brief The timestamp of the data.
     * 
     */
    uint64_t timestamp;

    /**
     * @brief Represents the payload of this data message.
     * 
     */
    std::vector<uint8_t> data;

    /**
     * @brief Construct a new Data Msg object.
     * 
     */
    DataMsg() : BaseMsg(MsgType::Data) {}

    /**
     * @brief Construct a new Data Msg object
     * 
     * @param sequence Sequence number this data is for.
     * @param entryId Entry ID of this data.
     * @param timestamp Timestamp of this data.
     * @param data Distributing data.
     */
    DataMsg(uint32_t sequence, uint16_t entryId, uint64_t timestamp, std::vector<uint8_t> data) 
        : BaseMsg(MsgType::Data), reqSequence(sequence), entryId(entryId), timestamp(timestamp), data(data) {}

    /**
     * @brief Move constructor.
     * 
     * @param other Source object.
     */
    DataMsg(DataMsg&& other)
        : BaseMsg(MsgType::Data)
        , reqSequence(other.reqSequence)
        , entryId(other.entryId)
        , timestamp(other.timestamp)
        , data(other.data) {}

    /**
     * @brief Move assignment operator.
     * 
     * @param other Source object.
     * @return DataMsg& lvalue.
     */
    DataMsg& operator=(DataMsg&& other)
    {
        reqSequence = other.reqSequence;
        entryId = other.entryId;
        timestamp = other.timestamp;
        data = other.data;
        return *this;
    }

    /**
     * @brief Serializes the data message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize()
    {
        uint8_t t = type;
        uint32_t len = data.size() + FIXED_LEN;
        uint8_t temp[len];

        serialisers::copyU8(temp, t);
        serialisers::copyU32(&temp[1], reqSequence);
        serialisers::copyU16(&temp[5], entryId);
        serialisers::copyU64(&temp[7], timestamp);
        memcpy(&temp[15], data.data(), data.size());

        return std::vector<uint8_t>(temp, temp + len);
    }

    /**
     * @brief Deserializes a bytes vector back into an data message.
     * 
     * @param bytes Bytes vector to deserialize.
     */
    void deserialize(std::vector<uint8_t>& bytes)
    {
        uint8_t* temp = bytes.data();
        uint8_t t = serialisers::getU8(&temp[0]);
        
        if (t != MsgType::Data)
        {
            throw std::invalid_argument("Message is not of MsgType::Data!");
        }
        reqSequence = serialisers::getU32(&temp[1]);
        entryId = serialisers::getU16(&temp[5]);
        timestamp = serialisers::getU64(&temp[7]);
        uint32_t dataLen = bytes.size() - FIXED_LEN;
        data = std::vector<uint8_t>(&temp[15], &temp[15] + dataLen);
    }
};

#endif // H_DATA_MSG