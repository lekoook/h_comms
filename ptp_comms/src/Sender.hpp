#ifndef H_SENDER
#define H_SENDER

#include <thread>
#include <mutex>
#include <condition_variable>
#include "TxQueueData.hpp"
#include "Packet.hpp"
#include "ATransceiver.hpp"
#include "SignalWaitManager.hpp"
#include "WaitTimer.hpp"

namespace ptp_comms
{

class Sender : public AWaiter
{
private:
    /**
     * @brief Default transmission rate (in Hz).
     * 
     */
    double const DEFAULT_TX_RATE = 20;

    /**
     * @brief Default transmission rate threshold (in Hz).
     * 
     */
    double const DEFAULT_TX_RATE_THRESHOLD = 100;

    /**
     * @brief The additive amount to increase the transmission rate (in Hz).
     * 
     */
    double const TX_RATE_ADDITIVE = 3;

    /**
     * @brief Lower bound of timeout duration.
     * 
     */
    uint32_t const MIN_TIMEOUT = 50;

    /**
     * @brief Maximum number of times to try and send a segment of a piece of data.
     * 
     */
    uint8_t const MAX_SEGMENT_TRIES = 7;

    /**
     * @brief Tracks the current transmission rate (in Hz).
     * 
     */
    double txRate = DEFAULT_TX_RATE;

    /**
     * @brief Tracks the threshold of the transmission rate (in Hz).
     * 
     */
    double txRateThreshold = DEFAULT_TX_RATE_THRESHOLD;

    /**
     * @brief Flag that indicates if there was at least a timeout once.
     * 
     */
    bool zeroTimeout = true;

    /**
     * @brief Tracks the number of times a timeout has occured.
     * 
     */
    uint32_t timeoutCnt = 0;

    /**
     * @brief Mutex to help with the signalling of reception of acknowledgement packets.
     * 
     */
    std::mutex mAck;

    /**
     * @brief Condition variable to help with the signalling of reception of acknowledgement packets.
     * 
     */
    std::condition_variable cvAck;

    /**
     * @brief Flag to help with the signalling of reception of acknowledgement packets.
     * 
     */
    bool gotAck;

    /**
     * @brief This will indicate for which source address we are waiting for the ACK.
     * 
     */
    std::string waitSrc;

    /**
     * @brief This will indicate for which port number we are waiting for the ACK.
     * 
     */
    uint16_t waitPort;

    /**
     * @brief This will indicate for which sequence number we are waiting for the ACK.
     * 
     */
    uint32_t waitSeq;

    /**
     * @brief This will indicate for which segment number we are waiting for the ACK.
     * 
     */
    uint32_t waitSeg;

    /**
     * @brief Contains the data and meta information to send.
     * 
     */
    TxQueueData txData;

    /**
     * @brief Thread that will execute the sending.
     * 
     */
    std::thread sendTh;
    
    /**
     * @brief Interface that performs the actual transmission.
     * 
     */
    ATransceiver* transceiver;

    /**
     * @brief Flag that indicates if a sending sequence is happening.
     * 
     */
    std::atomic<bool> running;

    /**
     * @brief Signal and wait manager to post waitings and get notified.
     * 
     */
    SignalWaitManager* signalManager;

    /**
     * @brief Timer used to wait for ACK.
     * 
     */
    WaitTimer ackWaitTimer;

    /**
     * @brief Waits for the acknowledgement (ACK) for a specific source address, sequence and segment number.
     * 
     * @param timeout The amount of time in milliseconds to wait for before giving up.
     * @return true If ACK was received.
     * @return false If ACK was never received.
     */
    bool _waitAck(uint32_t timeout)
    {
        ackWaitTimer.setTime(timeout / 1000.0f); // Convert from milliseconds to seconds.
        ackWaitTimer.wait();
        return ackWaitTimer.isInterrupted();
    }

    /**
     * @brief Sets the wait parameters for ACK.
     * 
     * @param seqNum Sequence number the ACK is to be for.
     * @param segNum Segment number the ACK is to be for.
     * @param src Source address the ACK should be coming from.
     * @param port Port number the ACK should be coming from.
     */
    void _setWait(uint32_t seqNum, uint32_t segNum, std::string src, uint16_t port)
    {
        signalManager->post(WaitID(seqNum, segNum), this);
    }

    /**
     * @brief Executes the actual sending.
     * 
     */
    bool _run()
    {
        int64_t dSize = txData.data.size();
        uint8_t* ptr = (uint8_t*)txData.data.data();
        uint64_t s = 0;
        uint32_t timeout = MIN_TIMEOUT;

        for (size_t i = 0; i < dSize; i+= Packet::MAX_SEGMENT_SIZE)
        {
            int tries = 0;
            auto last = std::min<int64_t>(dSize, (int64_t)(i + Packet::MAX_SEGMENT_SIZE));

            if (txData.dest == ptp_comms::BROADCAST_ADDR)
            {
                transceiver->sendTo(
                    Packet(txData.seqNum, s, dSize, std::vector<uint8_t>(ptr + i, ptr + last)).serialize(), 
                    txData.dest, 
                    txData.port);
                continue;
            }

            bool segFailed = false;
            while (tries < MAX_SEGMENT_TRIES)
            {
                _setWait(txData.seqNum, s, txData.dest, txData.port);
                if (transceiver->sendTo(
                            Packet(txData.seqNum, s, dSize, std::vector<uint8_t>(ptr + i, ptr + last)).serialize(), 
                            txData.dest, 
                            txData.port))
                {
                    ROS_INFO("Sent data for sequence %u to destination %s with port %u", txData.seqNum, txData.dest.c_str(), txData.port);
                }
                bool recvAck = _waitAck(timeout);
                tries++;
                
                if (recvAck)
                {
                    timeout *= 0.9;
                    if (timeout < MIN_TIMEOUT)
                    {
                        timeout = MIN_TIMEOUT;
                    }

                    if (txRate < txRateThreshold)
                    {
                        txRate *= 2.0;
                    }
                    else
                    {
                        txRate += TX_RATE_ADDITIVE;
                    }
                }
                else
                {
                    timeoutCnt++;
                    ROS_WARN_COND(timeoutCnt % 10 == 0, "Sequence %u has timeout %u times", txData.seqNum, timeoutCnt);

                    timeout *= 2.0;

                    if (tries >= MAX_SEGMENT_TRIES)
                    {
                        segFailed = true;
                    }

                    if (zeroTimeout)
                    {
                        zeroTimeout = false;
                        txRateThreshold /= 2.0;
                        txRate = DEFAULT_TX_RATE;
                    }
                    else
                    {
                        txRate /= 2.0;
                    }
                }
                ros::Rate(txRate).sleep();

                if (recvAck || segFailed)
                {
                    break;
                }
            }

            if (segFailed)
            {
                ROS_WARN("Sequence %u to address %s via port %u FAILED", 
                    txData.seqNum, txData.dest.c_str(), txData.port);
                return false;
            }

            s++;
        }
        running = false;
        return true;
    }

public:
    Sender(const TxQueueData& txData, ATransceiver* transceiver, SignalWaitManager* signalManager) 
        : txData(txData), transceiver(transceiver), signalManager(signalManager), running(false) {}

    ~Sender()
    {
        if (sendTh.joinable())
        {
            sendTh.join();
        }
    }

    /**
     * @brief Executes the sending sequence synchronously. Blocks until sending is finished.
     * 
     */
    bool execute()
    {
        running = true;
        bool res = _run();
        return res;
    }

    /**
     * @brief Executes the sending sequence asynchronously in a separate thread.
     * 
     */
    void executeAsync()
    {
        running = true;
        sendTh = std::thread(&Sender::_run, this);
    }

    bool isFinished()
    {
        return !running.load();
    }

    virtual void notify()
    {
        ackWaitTimer.interrupt();
    }
};

}

#endif // H_SENDER
