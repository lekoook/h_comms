#ifndef H_REQ_MACHINE
#define H_REQ_MACHINE

#include "ros/ros.h"
#include <cstdint>
#include <string>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "ReqStates.hpp"
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"
#include "ATransmitter.hpp"
#include "WaitTimer.hpp"

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
     * @brief Amount of time to wait before the wait timer elapses and callback is called.
     * 
     */
    const double WAIT_TIME = 5.0;

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

    /**
     * @brief Flag to indicate if this state machine has ended.
     * 
     */
    std::atomic<bool> isDestructed;

    /**
     * @brief Flag to indicate if a DATA message was received.
     * 
     */
    std::atomic<bool> receivedData;

    /**
     * @brief Sequence number of the ACK or DATA to wait for. This should not be set directly.
     * 
     */
    uint32_t waitSeq;

    /**
     * @brief EntryID of the ACK or DATA to wait for. This should not be set directly.
     * 
     */
    uint16_t waitEntryId;

    /**
     * @brief Mutex to protect the ACK or DATA waiting parameters.
     * 
     */
    std::mutex mWaitParams;

    /**
     * @brief Flag to determine if the the REQUEUE state was reached.
     * 
     */
    std::atomic<bool> needReqeue;

    /**
     * @brief Timer used to wait for ACK or DATA.
     * 
     */
    WaitTimer waitTimer;

    /**
     * @brief Sets the parameters of the ACK or DATA message to wait for. This method should be used to set the 
     * parameters instead of directly.
     * 
     * @param waitSeq Sequence number of ACK or DATA to wait for.
     * @param waitEntryId EntryID of ACK or DATA to wait for.
     */
    void _setWaitParams(uint32_t waitSeq, uint16_t waitEntryId);

    /**
     * @brief Checks against the already set parameters of ACK or DATA message to wait for. This method should be used 
     * to check the parameters instead of directly.
     * 
     * @param waitSeq Sequence number of ACK or DATA to wait for.
     * @param waitEntryId EntryID of ACK or DATA to wait for.
     * @return true If the parameters match.
     * @return false If the parameters do not match.
     */
    bool _checkWaitParams(uint32_t waitSeq, uint16_t waitEntryId);

    /**
     * @brief Starts the wait timer.
     * 
     */
    // void _startTimer();

    /**
     * @brief Stops the wait timer.
     * 
     */
    // void _stopTimer();

    /**
     * @brief Callback for the wait timer.
     * 
     * @param event Event information when the timer elapses.
     */
    // void _waitTimerCb(const ros::TimerEvent& event);

public:
    /**
     * @brief DATA message that was received containing the data.
     * 
     */
    DataMsg dataReceived;

    /**
     * @brief Queries if the state machine has ended the state sequences.
     * 
     * @return true If it has ended.
     * @return false If it has not yet ended.
     */
    bool hasEnded();

    /**
     * @brief Queries if the state machine passed through the REQUEUE state and requires a requeue of request.
     * 
     * @return true If requeue of request is needed.
     * @return false If requeue of request is not needed.
     */
    bool needRequeue();

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

    /**
     * @brief Queries if a DATA was actually received.
     * 
     * @return true If DATA was received.
     * @return false If DATA was not received.
     */
    bool hasReceived();
};

#endif // H_REQ_MACHINE