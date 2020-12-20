#ifndef H_REQ_MSG
#define H_REQ_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"
#include "serialisers.hpp"

/**
 * @brief Represents the request message for data distribution.
 * 
 */
class ReqMsg : public BaseMsg
{
private:
    /**
     * @brief Total length of fixed-length fields.
     * @details Length = Type(1) + RequestSequence(4) + RequestEntryID(2)
     * 
     */
    const uint8_t FIXED_LEN = 7;

public:
    /**
     * @brief Request sequence number field.
     * 
     */
    uint32_t reqSequence;

    /**
     * @brief The entry ID to request for.
     * 
     */
    uint16_t reqEntryId;

    /**
     * @brief Construct a new Req Msg object.
     * 
     */
    ReqMsg() : BaseMsg(Type::Request) {}

    /**
     * @brief Construct a new Req Msg object.
     * 
     * @param sequence The request sequence number of this message.
     * @param entryId The entry ID this request message is requesting for.
     */
    ReqMsg(uint32_t sequence, uint16_t entryId) : BaseMsg(Type::Request), reqSequence(sequence), reqEntryId(entryId) {}

    /**
     * @brief Serializes the request message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize()
    {
        uint8_t t = type;
        uint8_t temp[FIXED_LEN];

        serialisers::copyU8(temp, t);
        serialisers::copyU32(&temp[1], reqSequence);
        serialisers::copyU16(&temp[5], reqEntryId);

        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector back into a request message.
     * 
     * @param bytes Bytes vector to deserialize.
     */
    void deserialize(std::vector<uint8_t>& bytes)
    {
        uint8_t* temp = bytes.data();
        uint8_t t = serialisers::getU8(&temp[0]);
        
        if (t != Type::Request)
        {
            throw std::invalid_argument("Message is not of Type::Request!");
        }

        reqSequence = serialisers::getU32(&temp[1]);
        reqEntryId = serialisers::getU16(&temp[5]);
    }
};

#endif // H_REQ_MSG