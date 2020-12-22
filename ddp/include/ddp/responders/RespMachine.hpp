#ifndef H_RESP_MACHINE
#define H_RESP_MACHINE

#include <cstdint>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
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
     * @brief Maximum number of attempts to send DATA message.
     * 
     */
    const uint8_t MAX_SEND_TRIES = 3;

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

    /**
     * @brief Sequence number of the DATA to wait for. This should not be set directly.
     * 
     */
    uint32_t waitSeq;

    /**
     * @brief EntryID of the DATA to wait for. This should not be set directly.
     * 
     */
    uint16_t waitEntryId;

    /**
     * @brief Mutex to protect the DATA waiting parameters.
     * 
     */
    std::mutex mWaitParams;

    /**
     * @brief Flag to help determine a correct DATA has been received.
     * 
     */
    bool gotMsg = false;

    /**
     * @brief Mutex to protect DATA flag.
     * 
     */
    std::mutex mGotMsg;

    /**
     * @brief Condition variable to assist with signalling of receiving correct DATA.
     * 
     */
    std::condition_variable cvGotMsg;

    /**
     * @brief The number of attempts to send DATA message.
     * 
     */
    uint8_t sendTries = 0;

    /**
     * @brief Sets the parameters of the DATA message to wait for. This method should be used to set the 
     * parameters instead of directly.
     * 
     * @param waitSeq Sequence number of DATA to wait for.
     * @param waitEntryId EntryID of DATA to wait for.
     */
    void _setWaitParams(uint32_t waitSeq, uint16_t waitEntryId);

    /**
     * @brief Checks against the already set parameters of DATA message to wait for. This method should be used 
     * to check the parameters instead of directly.
     * 
     * @param waitSeq Sequence number of DATA to wait for.
     * @param waitEntryId EntryID of DATA to wait for.
     * @return true If the parameters match.
     * @return false If the parameters do not match.
     */
    bool _checkWaitParams(uint32_t waitSeq, uint16_t waitEntryId);

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