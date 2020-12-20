#ifndef H_BASE_MSG
#define H_BASE_MSG

#include <vector>
#include <cstdint>

/**
 * @brief Represents the base message for all message exchanges.
 * 
 */
class BaseMsg
{
protected:
    /**
     * @brief Represents the type of a message. Can be one of the following: advertisement, Acknowledgement, Request or Data.
     * 
     */
    enum Type
    {
        Advertisement = 0,
        Acknowledgement = 1,
        Request = 2,
        Data = 3
    };

    /**
     * @brief Specific type of this message.
     * 
     */
    const Type type;

    /**
     * @brief Construct a new Base Msg object.
     * 
     * @param type Specific type of this message.
     */
    BaseMsg(Type type) : type(type) {}

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