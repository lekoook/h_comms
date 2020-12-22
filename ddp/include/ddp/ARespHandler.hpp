#ifndef H_ARESP_HANDLER
#define H_ARESP_HANDLER

#include <string>
#include "messages/AckMsg.hpp"

/**
 * @brief Interface that provides the capability to handle incoming ACK messages meant for responders as well as
 * spawning a new responder to conduct data exchange.
 * 
 */
class ARespHandler
{
public:
    /**
     * @brief Notify a responder of an incoming ACK message.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    virtual void notifyAck(std::string src, AckMsg& ackMsg) = 0;

    /**
     * @brief Queue a new data response that will spawn a new responder at some point.
     * 
     * @param sequence Sequence number of the initial request.
     * @param entryId Entry ID in the MIT to respond.
     * @param respTarget Target robot this response is intended for.
     */
    virtual void queueResp(uint32_t sequence, uint16_t entryId, std::string respTarget) = 0;
};

#endif // H_ARESP_HANDLER