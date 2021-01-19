#ifndef H_AODV_RREQ_MSG
#define H_AODV_RREQ_MSG

#include <string>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "BaseMsg.hpp"
#include "ptp_comms/serialisers.hpp"

namespace aodv
{

class RreqMsg : public BaseMsg
{
private:
    /**
     * @brief Total of 24 bytes in one RREQ message.
     * 
     */
    uint8_t const FIXED_LEN = 24;

    /**
     * @brief Flag bit masks.
     * 
     */
    uint16_t const JOIN_FLAG_BIT = 0x8000;
    uint16_t const REPAIR_FLAG_BIT = 0x4000;
    uint16_t const GRATUITOUS_FLAG_BIT = 0x2000;
    uint16_t const DEST_ONLY_FLAG_BIT = 0x1000;
    uint16_t const UNK_SEQUENCE_FLAG_BIT = 0x800;

    /**
     * @brief The string address of the destination for which a route is desired.
     * 
     */
    std::string destAddr = "";

    /**
     * @brief The string address of the node which originated the Route Request.
     * 
     */
    std::string srcAddr = "";
    
public:
    /**
     * @brief Join flag; reserved for multicast.
     * 
     */
    bool isJoin = false;

    /**
     * @brief Repair flag; reserved for multicast.
     * 
     */
    bool isRepair = false;
    
    /**
     * @brief Gratuitous RREP flag; indicates whether a gratuitous RREP should be unicast to the node specified in the 
     * Destination IP Address field
     * 
     */
    bool isGratuitous = false;

    /**
     * @brief Destination only flag; indicates only the destination may respond to this RREQ.
     * 
     */
    bool isDestOnly = false;

    /**
     * @brief Unknown sequence number; indicates the destination sequence number is unknown.
     * 
     */
    bool isUnkSequence = false;

    /**
     * @brief The number of hops from the Originator IP Address to the node handling the request.
     * 
     */
    uint8_t hopCount = 0;

    /**
     * @brief A sequence number uniquely identifying the particular RREQ when taken in conjunction with the originating 
     * node's IP address.
     * 
     */
    uint32_t rreqId = 0;

    /**
     * @brief The latest sequence number received in the past by the originator for any route towards the destination.
     * 
     */
    uint32_t destSeq = 0;

    /**
     * @brief The current sequence number to be used in the route entry pointing towards the originator of the route 
     * request.
     * 
     */
    uint32_t srcSeq = 0;

    /**
     * @brief Construct a new Rreq Msg object.
     * 
     */
    RreqMsg() : BaseMsg(MsgType::RReq) {}

    /**
     * @brief Construct a new Rreq Msg object.
     * 
     * @param rreqId A sequence number uniquely identifying the particular RREQ when taken in conjunction with the 
     * originating node's IP address.
     * @param destAddr The string address of the destination for which a route is desired.
     * @param destSeq The latest sequence number received in the past by the originator for any route towards the 
     * destination.
     * @param srcAddr The string address of the node which originated the Route Request.
     * @param srcSeq The current sequence number to be used in the route entry pointing towards the originator of the 
     * route request.
     * @param isJoin Join flag; reserved for multicast.
     * @param isRepair Repair flag; reserved for multicast.
     * @param isGratuitous Gratuitous RREP flag; indicates whether a gratuitous RREP should be unicast to the node 
     * specified in the Destination IP Address field
     * @param isDestOnly Destination only flag; indicates only the destination may respond to this RREQ.
     * @param isUnkSequence Unknown sequence number; indicates the destination sequence number is unknown.
     */
    RreqMsg(uint32_t rreqId, std::string destAddr, uint32_t destSeq, std::string srcAddr, uint32_t srcSeq, 
        bool isJoin=false, bool isRepair=false, bool isGratuitous=false, bool isDestOnly=false, 
        bool isUnkSequence=false) 
        : BaseMsg(MsgType::RReq), rreqId(rreqId), destSeq(destSeq), srcSeq(srcSeq), isJoin(isJoin), isRepair(isRepair), 
        isGratuitous(isGratuitous), isDestOnly(isDestOnly), isUnkSequence(isUnkSequence)
    {
        setDestAddr(destAddr);
        setSrcAddr(srcAddr);
    }

    /**
     * @brief Sets the destination address.
     * 
     * @param destAddr Destination address.
     */
    void setDestAddr(std::string destAddr)
    {
        if (destAddr.size() > 4)
        {
            throw std::invalid_argument("Destination address cannot be longer than 4 characters.");
        }
        else
        {
            this->destAddr = destAddr;
        }
    }

    /**
     * @brief Gets the destination address.
     * 
     * @return std::string Destination address.
     */
    std::string getDestAddr()
    {
        return destAddr;
    }

    /**
     * @brief Sets the source address.
     * 
     * @param srcAddr Source address.
     */
    void setSrcAddr(std::string srcAddr)
    {
        if (srcAddr.size() > 4)
        {
            throw std::invalid_argument("Source address cannot be longer than 4 characters.");
        }
        else
        {
            this->srcAddr = srcAddr;
        }
    }

    /**
     * @brief Gets the source address.
     * 
     * @return std::string Source address.
     */
    std::string getSrcAddr()
    {
        return srcAddr;
    }

    /**
     * @brief Serializes the message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    virtual std::vector<uint8_t> serialize()
    {
        uint8_t temp[FIXED_LEN] = { 0 };
        serialisers::copyU8(&temp[0], 1);
        uint16_t flags = 0;
        flags |= isJoin ? JOIN_FLAG_BIT : 0;
        flags |= isRepair ? REPAIR_FLAG_BIT : 0;
        flags |= isGratuitous ? GRATUITOUS_FLAG_BIT : 0;
        flags |= isDestOnly ? DEST_ONLY_FLAG_BIT : 0;
        flags |= isUnkSequence ? UNK_SEQUENCE_FLAG_BIT : 0;
        serialisers::copyU16(&temp[1], flags);
        serialisers::copyU8(&temp[3], hopCount);
        serialisers::copyU32(&temp[4], rreqId);
        std::memcpy(&temp[8], destAddr.c_str(), destAddr.size());
        serialisers::copyU32(&temp[12], destSeq);
        std::memcpy(&temp[16], srcAddr.c_str(), srcAddr.size());
        serialisers::copyU32(&temp[20], srcSeq);
        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector into a RREQ message.
     * 
     * @param msg Bytes vector containing the message.
     */
    virtual void deserialize(std::vector<uint8_t> msg)
    {
        if (msg.size() != FIXED_LEN)
        {
            std::ostringstream oss;
            oss << "Message is not the correct length which is " << FIXED_LEN << " bytes.";
            throw std::invalid_argument(oss.str().c_str());
        }

        if ((MsgType)msg[0] != MsgType::RReq)
        {
            throw std::invalid_argument("Message is not the correct type which is RREQ type.");
        }

        uint8_t* temp = msg.data();
        uint16_t flags = serialisers::getU16(&temp[1]);
        isJoin = (flags & JOIN_FLAG_BIT) > 1;
        isRepair = (flags & REPAIR_FLAG_BIT) > 1;
        isGratuitous = (flags & GRATUITOUS_FLAG_BIT) > 1;
        isDestOnly = (flags & DEST_ONLY_FLAG_BIT) > 1;
        isUnkSequence = (flags & UNK_SEQUENCE_FLAG_BIT) > 1;
        hopCount = serialisers::getU8(&temp[3]);
        rreqId = serialisers::getU32(&temp[4]);
        destAddr = std::string((char*)(&temp[8]), std::strlen((char*)&temp[8]));
        destSeq = serialisers::getU32(&temp[12]);
        srcAddr = std::string((char*)(&temp[16]), std::strlen((char*)&temp[16]));
        srcSeq = serialisers::getU32(&temp[20]);
    }
};

}

#endif // H_AODV_RREQ_MSG