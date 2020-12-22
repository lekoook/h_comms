#ifndef H_ADV_MSG
#define H_ADV_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"
#include "serialisers.hpp"

/**
 * @brief Represents the advertisement message for data distribution.
 * 
 */
class AdvMsg : public BaseMsg
{
private:
    /**
     * @brief Total length of fixed-length fields.
     * @details Length = MsgType(1)
     * 
     */
    const uint8_t FIXED_LEN = 1;

public:
    /**
     * @brief Represents the payload of this advertisement message.
     * 
     */
    std::vector<uint8_t> data;

    /**
     * @brief Construct a new Adv Msg object.
     * 
     */
    AdvMsg() : BaseMsg(MsgType::Advertisement) {}

    /**
     * @brief Construct a new Adv Msg object.
     * 
     * @param advData Advertisement data.
     */
    AdvMsg(std::vector<uint8_t> advData) : BaseMsg(MsgType::Advertisement), data(advData) {}

    /**
     * @brief Move constructor.
     * 
     * @param other Source object.
     */
    AdvMsg(AdvMsg&& other)
        : BaseMsg(MsgType::Advertisement)
        , data(other.data) {}

    /**
     * @brief Move assignment operator.
     * 
     * @param other Source object.
     * @return AdvMsg& lvalue.
     */
    AdvMsg& operator=(AdvMsg&& other)
    {
        data = other.data;
        return *this;
    }

    /**
     * @brief Serializes the advertisement message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize()
    {
        uint8_t t = type;
        uint32_t len = data.size() + FIXED_LEN;
        uint8_t temp[len];

        serialisers::copyU8(temp, t);
        memcpy(&temp[1], data.data(), data.size());

        return std::vector<uint8_t>(temp, temp + len);
    }

    /**
     * @brief Deserializes a bytes vector back into an advertisement message.
     * 
     * @param bytes Bytes vector to deserialize.
     */
    void deserialize(std::vector<uint8_t>& bytes)
    {
        uint8_t* temp = bytes.data();
        uint8_t t = serialisers::getU8(&temp[0]);
        
        if (t != MsgType::Advertisement)
        {
            throw std::invalid_argument("Message is not of MsgType::Advertisement!");
        }

        uint32_t dataLen = bytes.size() - FIXED_LEN;
        data = std::vector<uint8_t>(&temp[1], &temp[1] + dataLen);
    }
};

#endif // H_ADV_MSG