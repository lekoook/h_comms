#ifndef H_TXER_TH
#define H_TXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "Packet.hpp"
#include "ATransceiver.hpp"

namespace ptp_comms
{

/**
 * @brief Represents a thread that processes data that needs to be transmitted out.
 * 
 */
class TxerTh
{
private:
    /**
     * @brief Represents an item in the queue for data to be transmitted.
     * 
     */
    class TxQueueData
    {
    public:
        /**
         * @brief Indicates the number of times this piece of data has tried to be transmitted.
         * 
         */
        int8_t tries = 0;

        /**
         * @brief Sequence number for this piece of data.
         * 
         */
        uint32_t seqNum = 0;

        /**
         * @brief Actual payload data to transmit.
         * 
         */
        std::vector<uint8_t> data;

        /**
         * @brief Destination of this piece of data.
         * 
         */
        std::string dest;

        /**
         * @brief Port number this piece of data should go to.
         * 
         */
        uint16_t port;

        /**
         * @brief Construct a new Tx Queue Data object.
         * 
         */
        TxQueueData () {}

        /**
         * @brief Construct a new Tx Queue Data object.
         * 
         * @param data Payload data.
         * @param dest Destination to send to.
         * @param port Port to send to.
         * @param seqNum Sequence number for this data.
         */
        TxQueueData (std::vector<uint8_t> data, std::string dest, uint16_t port, uint32_t seqNum) 
            : data(data), dest(dest), port(port), seqNum(seqNum)
        {}
    };

    // Constants
    /**
     * @brief Maximum size of the transmission queue.
     * 
     */
    uint8_t const MAX_QUEUE_SIZE = 10;

    /**
     * @brief Maximum number of times to try and send a piece of data in the queue.
     * 
     */
    uint8_t const MAX_QUEUE_TRIES = 3;

    /**
     * @brief Maximum number of times to try and send a segment of a piece of data.
     * 
     */
    uint8_t const MAX_SEGMENT_TRIES = 3;

    /**
     * @brief Tranmission thread that will run the actual data processing (preparation and transmission).
     * 
     */
    std::thread txThread;

    /**
     * @brief Flag to indicate if transmission thread should run.
     * 
     */
    std::atomic<bool> thRunning;

    /**
     * @brief First-In-First-Out transmission queue for storing pieces of data waiting to be transmitted.
     * 
     */
    std::queue<TxQueueData> txQ;

    /**
     * @brief Mutex to protect the transmission queue.
     * 
     */
    std::mutex mTxQ;

    /**
     * @brief Interface that performs the actual transmission.
     * 
     */
    ATransceiver* cc;

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
     * @brief Mutex to protect the acknowledgement signalling parameters.
     * 
     */
    std::mutex mWaitAck;

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
     * @brief This indicates the current sequence number running count. Incremented after queueing each piece of data.
     * 
     */
    uint32_t sequence = 0;

    std::condition_variable cvGotTx;
    std::mutex mGotTx;

    /**
     * @brief Executes the processing of data.
     * @details A piece of data is removed from the transmission queue and prepared for transmission. If the data fails
     * to transmit entirely and successfully, it will re-queued back into the transmission queue for fixed number of
     * times.
     * 
     */
    void _run()
    {
        while(thRunning.load())
        {
            TxQueueData dat;
            
            // We wait until a signal is given that a TX has arrived.
            {
                std::unique_lock<std::mutex> tLock(mGotTx);
                cvGotTx.wait(tLock,
                    [this] () -> bool
                    {
                        return !txQ.empty();
                    });
                
                // Get data from queue.
                dat = txQ.front();
                txQ.pop();
            }

            ptp_comms::Neighbor_M nb = cc->neighbors();
            if (((dat.dest != ptp_comms::BROADCAST_ADDR) && (nb.find(dat.dest) == nb.end())) 
                || !sendData(dat))
            {
                dat.tries++;
                if (dat.tries < MAX_QUEUE_TRIES)
                {
                    _queueOne(dat); // Re-queue the data back into the queue to be processed again later. Give up after MAX_QUEUE_TRIES attempts.
                }
            }
        }
    }

    /**
     * @brief Prepares and transmits a piece of data from the transmission queue.
     * @details Each piece of data is broken into segments and transmitted out one by one. After each transmission,
     * an attempt to wait for acknowledgement (ACK) will be made before the next segment is sent. 
     * If it fails to receive ACK, it will retry transmission up to a certain number of times. If this also fails, the
     * data will be given up on.
     * 
     * @param data Data to be transmitted.
     * @return true If data was transmitted entirely and successfully.
     * @return false If data failed to transmit after several retries.
     */
    bool sendData(const TxQueueData& data)
    {
        int64_t dSize = data.data.size();
        uint8_t* ptr = (uint8_t*)data.data.data();
        uint64_t s = 0;

        for (size_t i = 0; i < dSize; i+= Packet::MAX_SEGMENT_SIZE)
        {
            int tries = 0;
            auto last = std::min<int64_t>(dSize, (int64_t)(i + Packet::MAX_SEGMENT_SIZE));
            do
            {
                cc->sendTo(
                    Packet(data.seqNum, s, dSize, std::vector<uint8_t>(ptr + i, ptr + last)).serialize(), 
                    data.dest, 
                    data.port);
                tries++;
            } while (
                data.dest != ptp_comms::BROADCAST_ADDR
                && tries < MAX_SEGMENT_TRIES 
                && !_waitAck(data.seqNum, s, data.dest, data.port, 1000) 
                );

            if (tries >= MAX_SEGMENT_TRIES)
            {
                return false;
            }
            s++;
        }
        return true;
    }

    /**
     * @brief Inserts one piece of data into the transmission queue.
     * 
     * @param sendData Data to insert.
     * @return true If queue is not full and successful.
     * @return false If queue is already full.
     */
    bool _queueOne(TxQueueData& sendData)
    {
        if (txQ.size() < MAX_QUEUE_SIZE)
        {
            txQ.push(sendData);
            cvGotTx.notify_one();
            return true;
        }
        else
        {
            return false;
        }
    }

    // TODO: The waiting should be done using sim time or ros wall time instead of system time.
    /**
     * @brief Waits for the acknowledgement (ACK) for a specific source address, sequence and segment number.
     * 
     * @param seqNum Sequence number the ACK is to be for.
     * @param segNum Segment number the ACK is to be for.
     * @param src Source address the ACK should be coming from.
     * @param port Port number the ACK should be coming from.
     * @param timeout The amount of time in milliseconds to wait for before giving up.
     * @return true If ACK was received.
     * @return false If ACK was never received.
     */
    bool _waitAck(uint32_t seqNum, uint32_t segNum, std::string src, uint16_t port, uint32_t timeout)
    {
        {
            std::lock_guard<std::mutex> wLock(mWaitAck);
            waitSrc = src;
            waitPort = port;
            waitSeq = seqNum;
            waitSeg = segNum;
        }
        gotAck = false;
        std::unique_lock<std::mutex> lock(mAck);
        bool res = cvAck.wait_for(
            lock, 
            std::chrono::milliseconds(timeout),
            [this] () -> bool
                {
                    return gotAck;
                }
            );
        return res;
    }

    /**
     * @brief Signals that an acknowledgement (ACK) has been received.
     * @details It will check if the ACK that arrived is what we are looking for. If it is, appropriate flags will be
     * set and signal will be sent.
     * 
     * @param seqNum Sequence number of this incoming ACK.
     * @param segNum Segment number of this incoming ACK.
     * @param src Source address of this incoming ACK.
     * @param port Port number of this incoming ACK.
     */
    void _signalAck(uint32_t seqNum, uint32_t segNum, std::string src, uint16_t port)
    {
        uint32_t tseq;
        uint32_t tseg;
        std::string tsrc;
        uint16_t tport;
        {
            std::lock_guard<std::mutex> wLock(mWaitAck);
            tseq = waitSeq;
            tseg = waitSeg;
            tsrc = waitSrc;
            tport = waitPort;
        }

        if (seqNum == tseq && segNum == tseg && src == tsrc && port == tport)
        {
            std::lock_guard<std::mutex> lock(mAck);
            gotAck = true;
            cvAck.notify_one(); // Inform that an ACK has been received.
        }
    }
    
public:
    /**
     * @brief Construct a new Txer Th object.
     * 
     * @param cc Interface that performs the actual transmission.
     */
    TxerTh(ATransceiver* cc) : cc(cc)
    {
        thRunning.store(false);
    }
    
    /**
     * @brief Destroy the Txer Th object.
     * 
     */
    ~TxerTh()
    {
        stop();
    }
    
    /**
     * @brief Begins the operation of this thread. If the thread was already running, this does nothing.
     * 
     */
    void start()
    {
        if (thRunning.load())
        {
            return; // Don't start another thread if one already started.
        }
        thRunning.store(true);
        txThread = std::thread(&TxerTh::_run, this);
    }

    /**
     * @brief Stops the operation of this thread.
     * 
     */
    void stop()
    {
        thRunning.store(false);
        if (txThread.joinable())
        {
            txThread.join();
        }
    }

    /**
     * @brief Queues the data for transmission.
     * 
     * @param data Data to be transmitted.
     * @param dest Destination for data.
     * @param port Port for data.
     * @return true If queue is not full and successfully queued.
     * @return false If queue is already full.
     */
    bool sendOne(std::vector<uint8_t> data, std::string dest, uint16_t port)
    {
        std::lock_guard<std::mutex> lock(mGotTx);
        TxQueueData sendData(data, dest, port, sequence++);
        return _queueOne(sendData);
    }

    /**
     * @brief Notifies this thread about an incoming acknowledgement (ACK) message.
     * 
     * @param seqNum Sequence number of this ACK.
     * @param segNum Segment number of this ACK.
     * @param src Source address of this ACK.
     * @param port Port number of this ACK.
     */
    void notifyAck(uint32_t seqNum, uint32_t segNum, std::string src, uint16_t port)
    {
        _signalAck(seqNum, segNum, src, port);
    }
};

}

#endif // H_TXER_TH