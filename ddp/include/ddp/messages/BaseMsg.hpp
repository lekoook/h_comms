#ifndef H_BASE_MSG
#define H_BASE_MSG

#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>

/**
 * @brief Represents the type of a message. Can be one of the following: advertisement, Acknowledgement, Request or Data.
 * 
 */
enum MsgType
{
    Advertisement = 0,
    Acknowledgement = 1,
    Request = 2,
    Data = 3
};

/**
 * @brief Represents the base message for all message exchanges.
 * 
 */
class BaseMsg
{
protected:
    /**
     * @brief Specific type of this message.
     * 
     */
    const MsgType type;

    /**
     * @brief Construct a new Base Msg object.
     * 
     * @param type Specific type of this message.
     */
    BaseMsg(MsgType type) : type(type) {}

public:
    /**
     * @brief Serializes the message into bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes.
     */
    virtual std::vector<uint8_t> serialize() = 0;

    /**
     * @brief Deserializes a bytes vector back into a message.
     * 
     * @param bytes Bytes to deserialize.
     */
    virtual void deserialize(std::vector<uint8_t>& bytes) = 0;
};

#endif // H_BASE_MSG