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
     * @details Length = Type(1) + AckSequence(4) + AckEntryID(2)
     * 
     */
    const uint8_t FIXED_LEN = 7;

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
     * @brief Construct a new Ack Msg object.
     * 
     */
    AckMsg() : BaseMsg(Type::Acknowledgement) {}

    /**
     * @brief Construct a new Ack Msg object.
     * 
     * @param sequence The acknowledgement sequence number of this message.
     * @param entryId The entry ID this acknowledgement message is acknowledging to.
     */
    AckMsg(uint32_t sequence, uint16_t entryId) 
        : BaseMsg(Type::Acknowledgement), ackSequence(sequence), ackEntryId(entryId) {}

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
        serialisers::copyU32(&temp[1], ackSequence);
        serialisers::copyU16(&temp[5], ackEntryId);

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
        
        if (t != Type::Acknowledgement)
        {
            throw std::invalid_argument("Message is not of Type::Acknowledgement!");
        }

        ackSequence = serialisers::getU32(&temp[1]);
        ackEntryId = serialisers::getU16(&temp[5]);
    }
};

#endif // H_ACK_MSG