#ifndef H_REQ_MACHINE
#define H_REQ_MACHINE

#include <cstdint>
#include <string>
#include "ReqStates.hpp"
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"
#include "ATransmitter.hpp"

// Forward declaration of required class.
class ReqState;

/**
 * @brief Represents the state machine of a Requestor's life.
 * 
 */
class ReqMachine
{
    friend class ReqState;
    friend class StartReqState;
    friend class WaitAckReqState;
    friend class WaitDataReqState;
    friend class SendAckReqState;
    friend class RequeueReqState;
    friend class DestructReqState;

private:
    /**
     * @brief Sequence number of this request.
     * 
     */
    uint32_t reqSequence;

    /**
     * @brief Entry ID this request is made for.
     * 
     */
    uint16_t reqEntryId;

    /**
     * @brief Intended target address this request is to be made to.
     * 
     */
    std::string reqTarget;

    /**
     * @brief The current state of this state machine.
     * 
     */
    ReqState* currentState;

    /**
     * @brief The next state of this state machine.
     * 
     */
    ReqState* nextState;

    /**
     * @brief Interface used to send messages.
     * 
     */
    ATransmitter* transmitter;

public:
    bool isDestructed = false;

    /**
     * @brief Construct a new Req Machine object.
     * 
     * @param reqSequence Sequence number of this request.
     * @param reqEntryId Entry ID this request is made for.
     * @param reqTarget Intended target address this request is to be made to.
     * @param transmitter Interface used to send messages.
     */
    ReqMachine(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, ATransmitter* transmitter);

    /**
     * @brief Destroy the Req Machine object.
     * 
     */
    ~ReqMachine();

    /**
     * @brief Execute the current state.
     * 
     */
    void run();

    /**
     * @brief Check if the next state can be transited to.
     * 
     */
    void checkTransit();

    /**
     * @brief Notifies the current state of a received ACK message.
     * 
     * @param ackMsg ACK message received.
     * @param src Source address of this message.
     */
    void recvAck(AckMsg& ackMsg, std::string src);

    /**
     * @brief Notifies the current state of a received DATA message.
     * 
     * @param dataMsg DATA message received.
     * @param src Source address of this message.
     */
    void recvData(DataMsg& dataMsg, std::string src);
};

#endif // H_REQ_MACHINE