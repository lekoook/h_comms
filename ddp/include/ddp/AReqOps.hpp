#ifndef H_AREQ_OPS
#define H_AREQ_OPS

#include <string>
#include "messages/ReqMsg.hpp"
#include "messages/AckMsg.hpp"

/**
 * @brief Interface for Requestors to send ACK or REQ messages and notify a timeout of data exchange (and stop data 
 * exchange).
 * 
 */
class AReqOps
{
public:
    /**
     * @brief Sends an ACK message out.
     * 
     * @param dest Intended destination address of ACK message.
     * @param ackMsg ACK message to send out.
     * @return true If sending was successful.
     * @return false If sending was not successful.
     */
    virtual bool sendAck(std::string dest, AckMsg& ackMsg) = 0;

    /**
     * @brief Sends a REQ message out.
     * 
     * @param dest Intended destination address of ACK message.
     * @param reqMsg REQ message to send out.
     * @return true If sending was successful.
     * @return false If sending was not successful.
     */
    virtual bool sendReq(std::string dest, ReqMsg& reqMsg) = 0;

    /**
     * @brief Notify that a timeout has occured and will stop data exchange.
     * 
     * @param reqSequence Request sequence number for which this timeout occured for.
     */
    virtual void notifyTimeout(uint32_t reqSequence) = 0;
};

#endif // H_AREQ_OPS