#ifndef H_AODV_BASE_MSG
#define H_AODV_BASE_MSG

#include <cstdint>
#include <vector>

namespace aodv
{

/**
 * @brief Indicates which AODV message type.
 * 
 */
enum MsgType
{
    RReq = 1,
    RRep = 2,
    RRer = 3,
    RRepAck = 4
};

/**
 * @brief Base message for all AODV message types.
 * 
 */
class BaseMsg
{
public:
    /**
     * @brief AODV message type.
     * 
     */
    MsgType msgType;

    /**
     * @brief Construct a new Base Msg object.
     * 
     */
    BaseMsg() {}

    /**
     * @brief Construct a new Base Msg object.
     * 
     * @param msgType AODV message type.
     */
    BaseMsg(MsgType msgType) : msgType(msgType) {}

    /**
     * @brief Serializes the message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    virtual std::vector<uint8_t> serialize()
    {
        return std::vector<uint8_t>({(uint8_t)msgType});
    }
    
    /**
     * @brief Deserializes a bytes vector into a base message.
     * 
     * @param msg Bytes vector containing the message.
     */
    virtual void deserialize(std::vector<uint8_t> msg)
    {
        if (msg.size() < 1) // There must at least contain the first byte which is the type.
        {
            return;
        }
        msgType = (MsgType)msg[0];
    }
};

}

#endif // H_BASE_MSG