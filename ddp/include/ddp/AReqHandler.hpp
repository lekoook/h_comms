#ifndef H_AREQ_HANDLER
#define H_AREQ_HANDLER

#include <string>
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"

/**
 * @brief Interface that provides the capability to handle incoming ACK or DATA messages meant for requestors as well as
 * spawning a new requestor to conduct data exchange.
 * 
 */
class AReqHandler
{
public:
    /**
     * @brief Notify a requestor of an incoming ACK message.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    virtual void notifyAck(std::string src, AckMsg& ackMsg) = 0;

    /**
     * @brief Notify a requestor of an incoming DATA message.
     * 
     * @param src Source address of DATA message.
     * @param dataMsg DATA message.
     */
    virtual void notifyData(std::string src, DataMsg& dataMsg) = 0;

    /**
     * @brief Queue a new data request that will spawn a new requestor at some point.
     * 
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     */
    virtual void queueReq(uint16_t entryId, std::string reqTarget) = 0;
};

#endif // H_AREQ_HANDLER