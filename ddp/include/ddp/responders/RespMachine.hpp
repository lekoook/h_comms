#ifndef H_RESP_MACHINE
#define H_RESP_MACHINE

#include <cstdint>
#include <string>
#include <atomic>
#include "RespStates.hpp"
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"
#include "ATransmitter.hpp"

// Forward declaration of required class.
class RespState;

/**
 * @brief Represents the state machine of a Responder's life.
 * 
 */
class RespMachine
{
    friend class RespState;
    friend class StartRespState;
    friend class PrepRespState;
    friend class SendDataRespState;
    friend class WaitAckDataRespState;
    friend class NoDataRespState;
    friend class DestructRespState;

private:
    /**
     * @brief Sequence number of this response.
     * 
     */
    uint32_t respSequence;

    /**
     * @brief Entry ID this response is made for.
     * 
     */
    uint16_t respEntryId;

    /**
     * @brief Intended target address this response is for.
     * 
     */
    std::string respTarget;

    /**
     * @brief The current state of this state machine.
     * 
     */
    RespState* currentState;

    /**
     * @brief The next state of this state machine.
     * 
     */
    RespState* nextState;

    /**
     * @brief Interface used to send messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Flag to indicate if this state machine has ended.
     * 
     */
    std::atomic<bool> isDestructed;

public:
    /**
     * @brief Queries if the state machine has ended the state sequences.
     * 
     * @return true If it has ended.
     * @return false If it has not yet ended.
     */
    bool hasEnded();

    /**
     * @brief Construct a new Resp Machine object.
     * 
     * @param respSequence Sequence number of this response.
     * @param respEntryId Entry ID this response is made for.
     * @param respTarget Intended target address this response is for.
     * @param transmitter Interface used to send messages.
     */
    RespMachine(uint32_t respSequence, uint16_t respEntryId, std::string respTarget, ATransmitter* transmitter);

    /**
     * @brief Destroy the Resp Machine object.
     * 
     */
    ~RespMachine();

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
};

#endif // H_RESP_MACHINE