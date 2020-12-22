#ifndef H_ACK_MSG
#define H_ACK_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"
#include "serialisers.hpp"

/**
 * @brief Represents the acknowledgement message for data distribution.
 * 
 */
class AckMsg : public BaseMsg
{
private:
    /**
     * @brief Total length of fixed-length fields.
     * @details Length = Type(1) + ForReq(1) + AckSequence(4) + AckEntryID(2)
     * 
     */
    const uint8_t FIXED_LEN = 8;

public:
    /**
     * @brief Acknowledge sequence number field.
     * 
     */
    uint32_t ackSequence;

    /**
     * @brief The entry ID to acknowledge to.
     * 
     */
    uint16_t ackEntryId;

    /**
     * @brief Field to indicate if the acknowledgement is for a request or data message.
     * @details This can only be one of two states. If true, it is acknowledgement for a request message. Otherwise 
     * false indicates that it is for a data message.
     * 
     */
    bool forReq = true;

    /**
     * @brief Construct a new Ack Msg object.
     * 
     */
    AckMsg() : BaseMsg(MsgType::Acknowledgement) {}

    /**
     * @brief Construct a new Ack Msg object.
     * 
     * @param sequence The acknowledgement sequence number of this message.
     * @param entryId The entry ID this acknowledgement message is acknowledging to.
     * @param forReq (Optional) If true, this is acknowledgement for request message. Otherwise it is for data message.
     */
    AckMsg(uint32_t sequence, uint16_t entryId, bool forReq=true) 
        : BaseMsg(MsgType::Acknowledgement), ackSequence(sequence), ackEntryId(entryId), forReq(forReq) {}

    /**
     * @brief Move constructor.
     * 
     * @param other Source object.
     */
    AckMsg(AckMsg&& other)
        : BaseMsg(MsgType::Acknowledgement)
        , ackSequence(other.ackSequence)
        , ackEntryId(other.ackEntryId) {}

    /**
     * @brief Move assignment operator.
     * 
     * @param other Source object.
     * @return AckMsg& lvalue.
     */
    AckMsg& operator=(AckMsg&& other)
    {
        ackSequence = other.ackSequence;
        ackEntryId = other.ackEntryId;
        return *this;
    }

    /**
     * @brief Serializes the acknowledgement message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize()
    {
        uint8_t t = type;
        uint8_t temp[FIXED_LEN];

        serialisers::copyU8(temp, t);
        serialisers::copyU8(&temp[1], (uint8_t)forReq);
        serialisers::copyU32(&temp[2], ackSequence);
        serialisers::copyU16(&temp[6], ackEntryId);

        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector back into an acknowledgement message.
     * 
     * @param bytes Bytes vector to deserialize.
     */
    void deserialize(std::vector<uint8_t>& bytes)
    {
        uint8_t* temp = bytes.data();
        uint8_t t = serialisers::getU8(&temp[0]);
        
        if (t != MsgType::Acknowledgement)
        {
            throw std::invalid_argument("Message is not of Type::Acknowledgement!");
        }

        forReq = serialisers::getU8(&temp[1]);
        ackSequence = serialisers::getU32(&temp[2]);
        ackEntryId = serialisers::getU16(&temp[6]);
    }
};

#endif // H_ACK_MSG