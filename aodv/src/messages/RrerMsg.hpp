#ifndef H_AODV_RRER_MSG
#define H_AODV_RRER_MSG

#include <string>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "BaseMsg.hpp"
#include "ptp_comms/serialisers.hpp"

namespace aodv
{

class RrerMsg : public BaseMsg
{
private:
    /**
     * @brief Total of 20 bytes in one RREP message.
     * 
     */
    uint8_t const FIXED_LEN = 20;

    /**
     * @brief Flag bit mask.
     * 
     */
    uint16_t const NO_DELETE_FLAG_BIT = 0x8000;

    /**
     * @brief The number of unreachable destinations included in the message; MUST be at least 1.
     * 
     */
    uint8_t destCount = 1;

    /**
     * @brief The string address of the destination that has become unreachable due to a link break.
     * 
     */
    std::string unrDestAddr = "";

    /**
     * @brief Additional (if needed) string address of the destination that has become unreachable due to a link break.
     * 
     */
    std::string addUnrDestAddr = "";

public:
    /**
     * @brief No delete flag; set when a node has performed a local repair of a link, and upstream nodes should not 
     * delete the route.
     * 
     */
    bool isNoDelete = false;

    /**
     * @brief The sequence number in the route table entry for the destination listed in the previous Unreachable 
     * Destination IP Address field.
     * 
     */
    uint32_t unrDestSeq = 0;

    /**
     * @brief Additional sequence number in the route table entry for the destination listed in the previous Unreachable 
     * Destination IP Address field.
     * 
     */
    uint32_t addUnrDestSeq = 0;

    /**
     * @brief Construct a new Rrer Msg object.
     * 
     */
    RrerMsg() : BaseMsg(MsgType::RRer) {}

    /**
     * @brief Construct a new Rrer Msg object.
     * 
     * @param unrDestAddr The string address of the destination that has become unreachable due to a link break.
     * @param unrDestSeq The sequence number in the route table entry for the destination listed in the previous 
     * Unreachable Destination IP Address field.
     * @param destCount The number of unreachable destinations included in the message; MUST be at least 1.
     * @param isNoDelete No delete flag; set when a node has performed a local repair of a link, and upstream nodes 
     * should not delete the route.
     */
    RrerMsg(std::string unrDestAddr, uint32_t unrDestSeq, uint8_t destCount, bool isNoDelete=false) 
        : BaseMsg(MsgType::RRer), unrDestSeq(unrDestSeq), isNoDelete(isNoDelete)
    {
        setUnrDestAddr(unrDestAddr);
        setDestCount(destCount);
    }

    RrerMsg(std::string unrDestAddr, uint32_t unrDestSeq, std::string addUnrDestAddr, uint32_t addUnrDestSeq, 
        uint8_t destCount, bool isNoDelete=false) 
        : BaseMsg(MsgType::RRer), addUnrDestSeq(addUnrDestSeq), isNoDelete(isNoDelete)
    {
        setUnrDestAddr(unrDestAddr);
        setAddUnrDestAddr(addUnrDestAddr);
        setDestCount(destCount);
    }

    /**
     * @brief Sets the unreachable destination address.
     * 
     * @param unrDestAddr Destination address.
     */
    void setUnrDestAddr(std::string unrDestAddr)
    {
        if (unrDestAddr.size() > 4)
        {
            throw std::invalid_argument("Destination address cannot be longer than 4 characters.");
        }
        else
        {
            this->unrDestAddr = unrDestAddr;
        }
    }

    /**
     * @brief Gets the unreachable destination address.
     * 
     * @return std::string Destination address.
     */
    std::string getUnrDestAddr()
    {
        return unrDestAddr;
    }

    /**
     * @brief Sets the additional unreachable destination address.
     * 
     * @param addUnrDestAddr Additional unreachable destination address.
     */
    void setAddUnrDestAddr(std::string addUnrDestAddr)
    {
        if (addUnrDestAddr.size() > 4)
        {
            throw std::invalid_argument("Destination address cannot be longer than 4 characters.");
        }
        else
        {
            this->addUnrDestAddr = addUnrDestAddr;
        }
    }

    /**
     * @brief Gets the additional unreachable destination address.
     * 
     * @return std::string Additional unreachable destination address.
     */
    std::string getAddUnrDestAddr()
    {
        return addUnrDestAddr;
    }

    /**
     * @brief Sets the destination count.
     * 
     * @param destCount Destination count.
     */
    void setDestCount(uint8_t destCount)
    {
        if (destCount < 1)
        {
            throw std::invalid_argument("Destination count out of range, need AT LEAST 1.");
        }
        this->destCount = destCount;
    }

    /**
     * @brief Gets the destination count.
     * 
     * @return uint8_t Destination count.
     */
    uint8_t getDestCount()
    {
        return destCount;
    }

    /**
     * @brief Serializes the message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    virtual std::vector<uint8_t> serialize()
    {
        uint8_t temp[FIXED_LEN] = { 0 };
        serialisers::copyU8(&temp[0], 3);
        uint16_t mid16 = isNoDelete ? NO_DELETE_FLAG_BIT : 0;
        serialisers::copyU16(&temp[1], mid16);
        serialisers::copyU8(&temp[3], destCount);
        std::memcpy(&temp[4], unrDestAddr.c_str(), unrDestAddr.size());
        serialisers::copyU32(&temp[8], unrDestSeq);
        std::memcpy(&temp[12], addUnrDestAddr.c_str(), addUnrDestAddr.size());
        serialisers::copyU32(&temp[16], addUnrDestSeq);
        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector into a RRER message.
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

        if ((MsgType)msg[0] != MsgType::RRer)
        {
            std::invalid_argument("Message is not the correct type which is RRER type.");
        }

        uint8_t* temp = msg.data();
        uint16_t mid16 = serialisers::getU16(&temp[1]);
        isNoDelete = (mid16 & NO_DELETE_FLAG_BIT) > 1;
        destCount = serialisers::getU8(&temp[3]);
        unrDestAddr = std::string((char*)(&temp[4]), std::strlen((char*)&temp[4]));
        unrDestSeq = serialisers::getU32(&temp[8]);
        addUnrDestAddr = std::string((char*)(&temp[12]), std::strlen((char*)&temp[12]));
        addUnrDestSeq = serialisers::getU32(&temp[16]);
    }
};

}

#endif // H_AODV_RRER_MSG