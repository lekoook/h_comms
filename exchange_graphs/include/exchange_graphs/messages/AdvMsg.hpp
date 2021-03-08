#ifndef H_ADV_MSG
#define H_ADV_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"

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
     * @brief Construct a new Adv Msg object.
     * 
     */
    AdvMsg() : BaseMsg(MsgType::Advertisement) {}

    /**
     * @brief Serializes the advertisement message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize() { return {type}; }

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
    }
};

#endif // H_ADV_MSG
