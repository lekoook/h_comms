#ifndef H_AODV_RREP_MSG
#define H_AODV_RREP_MSG

#include <string>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "BaseMsg.hpp"
#include "ptp_comms/serialisers.hpp"

namespace aodv
{

class RrepMsg : public BaseMsg
{
private:
    /**
     * @brief Total of 20 bytes in one RREP message.
     * 
     */
    uint8_t const FIXED_LEN = 20;

    /**
     * @brief Flag bit masks.
     * 
     */
    uint16_t const REPAIR_FLAG_BIT = 0x8000;
    uint16_t const ACK_FLAG_BIT = 0x4000;

    /**
     * @brief The string address of the destination for which a route is supplied.
     * 
     */
    std::string destAddr = "";

    /**
     * @brief The string address of the node which originated the RREQ for which the route is supplied.
     * 
     */
    std::string srcAddr = "";

    /**
     * @brief If nonzero, the 5-bit Prefix Size specifies that the indicated next hop may be used for any nodes with the
     *  same routing prefix (as defined by the Prefix Size) as the requested destination.
     * 
     */
    uint8_t prefixSize = 0;

public:
    /**
     * @brief Repair flag; reserved for multicast.
     * 
     */
    bool isRepair = false;

    /**
     * @brief Acknowledgment required.
     * 
     */
    bool isAckRequired = false;

    /**
     * @brief The number of hops from the Originator IP Address to the Destination IP Address.  For multicast route 
     * requests this indicates the number of hops to the multicast tree member sending the RREP.
     * 
     */
    uint8_t hopCount = 0;

    /**
     * @brief The destination sequence number associated to the route.
     * 
     */
    uint32_t destSeq = 0;

    /**
     * @brief The time in milliseconds for which nodes receiving the RREP consider the route to be valid.
     * 
     */
    uint32_t lifetime = 0;

    /**
     * @brief Construct a new Rrep Msg object.
     * 
     */
    RrepMsg() : BaseMsg(MsgType::RRep) {}

    /**
     * @brief Construct a new Rrep Msg object.
     * 
     * @param destAddr The string address of the destination for which a route is supplied.
     * @param destSeq The destination sequence number associated to the route.
     * @param srcAddr The string address of the node which originated the RREQ for which the route is supplied.
     * @param lifetime The time in milliseconds for which nodes receiving the RREP consider the route to be valid.
     * @param prefixSize If nonzero, the 5-bit Prefix Size specifies that the indicated next hop may be used for any 
     * nodes with the same routing prefix (as defined by the Prefix Size) as the requested destination.
     * @param isRepair Repair flag; reserved for multicast.
     * @param isAckRequired Acknowledgment required.
     */
    RrepMsg(std::string destAddr, uint32_t destSeq, std::string srcAddr, uint32_t lifetime, uint8_t prefixSize, 
        bool isRepair=false, bool isAckRequired=false) 
        : BaseMsg(MsgType::RRep), destSeq(destSeq), lifetime(lifetime), isRepair(isRepair), isAckRequired(isAckRequired)
    {
        setDestAddr(destAddr);
        setSrcAddr(srcAddr);
        setPrefixSize(prefixSize);
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
     * @brief Sets the prefix size.
     * 
     * @param prefixSize Prefix size.
     */
    void setPrefixSize(uint8_t prefixSize)
    {
        if ((prefixSize & 0xE0) > 0) // Make sure prefix size can fit within 5 bits.
        {
            throw std::invalid_argument("Prefix size out of range for a 5 bits usigned int field.");
        }
        this->prefixSize = prefixSize;
    }

    /**
     * @brief Gets the prefix size.
     * 
     * @return uint8_t Prefix size.
     */
    uint8_t getPrefixSize()
    {
        return prefixSize;
    }

    /**
     * @brief Serializes the message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    virtual std::vector<uint8_t> serialize()
    {
        uint8_t temp[FIXED_LEN] = { 0 };
        serialisers::copyU8(&temp[0], 2);
        uint16_t mid16 = 0;
        mid16 |= isRepair ? REPAIR_FLAG_BIT : 0;
        mid16 |= isAckRequired ? ACK_FLAG_BIT : 0;
        mid16 |= (prefixSize & 0x1F); // Sanity check, only lowest 5 bits is valid.
        serialisers::copyU16(&temp[1], mid16);
        serialisers::copyU8(&temp[3], hopCount);
        std::memcpy(&temp[4], destAddr.c_str(), destAddr.size());
        serialisers::copyU32(&temp[8], destSeq);
        std::memcpy(&temp[12], srcAddr.c_str(), srcAddr.size());
        serialisers::copyU32(&temp[16], lifetime);
        return std::vector<uint8_t>(temp, temp + FIXED_LEN);
    }

    /**
     * @brief Deserializes a bytes vector into a RREP message.
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

        if ((MsgType)msg[0] != MsgType::RRep)
        {
            std::invalid_argument("Message is not the correct type which is RREP type.");
        }

        uint8_t* temp = msg.data();
        uint16_t mid16 = serialisers::getU16(&temp[1]);
        isRepair = (mid16 & REPAIR_FLAG_BIT) > 1;
        isAckRequired = (mid16 & ACK_FLAG_BIT) > 1;
        prefixSize = (serialisers::getU8(&temp[2]) & 0x1F); // Sanity check, only lowest 5 bits is valid.
        hopCount = serialisers::getU8(&temp[3]);
        destAddr = std::string((char*)(&temp[4]), std::strlen((char*)&temp[4]));
        destSeq = serialisers::getU32(&temp[8]);
        srcAddr = std::string((char*)(&temp[12]), std::strlen((char*)&temp[12]));
        lifetime = serialisers::getU32(&temp[16]);
    }
};

}

#endif // H_AODV_RREP_MSG