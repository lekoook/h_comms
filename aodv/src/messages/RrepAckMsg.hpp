#ifndef H_AODV_RREP_ACK_MSG
#define H_AODV_RREP_ACK_MSG

#include <string>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "BaseMsg.hpp"
#include "ptp_comms/serialisers.hpp"

namespace aodv
{

class RrepAckMsg : public BaseMsg
{
private:
    /**
     * @brief Total of 2 bytes in one RREP-ACK message.
     * 
     */
    uint8_t const FIXED_LEN = 2;

public:
    /**
     * @brief Construct a new Rrep Ack Msg object.
     * 
     */
    RrepAckMsg() : BaseMsg(MsgType::RRepAck) {}

    /**
     * @brief Serializes the message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    virtual std::vector<uint8_t> serialize()
    {
        uint8_t temp[FIXED_LEN] = { 0 };
        serialisers::copyU8(&temp[0], 4);
        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector into a RREP-ACK message.
     * 
     * @param msg Bytes vector containing the message.
     */
    virtual void deserialize(std::vector<uint8_t> msg)
    {
        if (msg.size() != FIXED_LEN)
        {
            std::ostringstream oss;
            oss << "Message is not the correct length which is " << FIXED_LEN << " bytes.";
            std::invalid_argument(oss.str().c_str());
        }

        if ((MsgType)msg[0] != MsgType::RRepAck)
        {
            std::invalid_argument("Message is not the correct type which is RREP-ACK type.");
        }
    }
};

}

#endif // H_AODV_RREP_ACK_MSG