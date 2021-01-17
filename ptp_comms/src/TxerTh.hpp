#ifndef H_TXER_TH
#define H_TXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include <memory>
#include <condition_variable>
#include "Packet.hpp"
#include "ATransceiver.hpp"
#include "TxQueueData.hpp"
#include "Sender.hpp"
#include "SignalWaitManager.hpp"

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
     * @brief Comparator function for TxQueueData.
     * 
     */
    struct TxCompare
    {
        bool operator()(const TxQueueData& first, const TxQueueData& second)
        {
            return first.data.size() > second.data.size();
        }
    };

    // Constants
    /**
     * @brief Maximum size of the transmission queue.
     * 
     */
    uint8_t const MAX_QUEUE_SIZE = 30;

    /**
     * @brief Maximum number of times to try and send a piece of data in the queue.
     * 
     */
    uint8_t const MAX_QUEUE_TRIES = 3;

    /**
     * @brief Maximum number of senders that are sending long data length.
     * 
     */
    uint8_t const MAX_LONG_SENDERS = 3;

    /**
     * @brief Threshold that determines if a single piece of data is small or large. In bytes.
     * 
     */
    uint32_t const SIZE_THRESHOLD = 100000;

    /**
     * @brief Frequency of servicing large data.
     * 
     */
    double const LARGE_TX_FREQ = 20.0;

    /**
     * @brief Tranmission thread that will run the actual data processing (preparation and transmission).
     * 
     */
    std::thread txThread;

    /**
     * @brief Tranmission thread that will run the actual data processing (preparation and transmission) for LARGE data.
     * @details Whether a data or not is determined by \p SIZE_THRESHOLD.
     * 
     */
    std::thread largeTxThread;

    /**
     * @brief Flag to indicate if transmission thread should run.
     * 
     */
    std::atomic<bool> thRunning;

    /**
     * @brief Prioritized transmission queue for storing pieces of small sized data waiting to be transmitted.
     * 
     */
    std::priority_queue<TxQueueData, std::vector<TxQueueData>, TxCompare> smallTxQ;

    /**
     * @brief Prioritized transmission queue for storing pieces of large sized data waiting to be transmitted.
     * 
     */
    std::priority_queue<TxQueueData, std::vector<TxQueueData>, TxCompare> largeTxQ;

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
     * @brief This indicates the current sequence number running count. Incremented after queueing each piece of data.
     * 
     */
    uint32_t sequence = 0;

    /**
     * @brief To signal when a new small data piece is queued and ready.
     * 
     */
    std::condition_variable cvGotTx;

    /**
     * @brief Mutex to protect small data queue.
     * 
     */
    std::mutex mSmallTxQ;

    /**
     * @brief Mutex to protect large data queue.
     * 
     */
    std::mutex mLargeTxQ;

    /**
     * @brief Record of all existing senders with long data length.
     * 
     */
    std::map<uint32_t, std::unique_ptr<Sender>> longSenders;

    /**
     * @brief Signalling manager to manage all waiters and signallers.
     * 
     */
    SignalWaitManager sigMgr;

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
                std::unique_lock<std::mutex> tLock(mSmallTxQ);
                cvGotTx.wait(tLock,
                    [this] () -> bool
                    {
                        return !smallTxQ.empty();
                    });
                dat = smallTxQ.top();
                smallTxQ.pop();
            }

            // We now check small data queue and execute syncly (if any).
            {
                ptp_comms::Neighbor_M nb = cc->neighbors();
                if (dat.dest == ptp_comms::BROADCAST_ADDR || nb.find(dat.dest) != nb.end())
                {
                    Sender sender(dat, cc, &sigMgr);
                    if (!sender.execute())
                    {
                        dat.tries++;
                        if (dat.tries < MAX_QUEUE_TRIES)
                        {
                            _queueOne(dat); // Re-queue the data back into the queue to be processed again later. Give up after MAX_QUEUE_TRIES attempts.
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief Executes the processing of large data.
     * 
     */
    void _runLarge()
    {
        while (thRunning.load())
        {
            // Clean any long senders that have finished.
            for (auto it = longSenders.begin(); it != longSenders.end();)
            {
                bool res = it->second->isFinished();
                if (res)
                {
                    it = longSenders.erase(it);
                }
                else
                {
                    it++;
                }
            }
            
            // We check large data queue first and execute them asyncly (if any).
            TxQueueData dat;
            bool have = false;
            {
                std::lock_guard<std::mutex> lock(mLargeTxQ);
                if (!largeTxQ.empty() && longSenders.size() < MAX_LONG_SENDERS)
                {
                    dat = largeTxQ.top();
                    largeTxQ.pop();
                    have = true;
                }
            }

            // TODO: Re-queue those data that were not sent successfully.
            if (have)
            {
                ptp_comms::Neighbor_M nb = cc->neighbors();
                if (dat.dest == ptp_comms::BROADCAST_ADDR || nb.find(dat.dest) != nb.end())
                {
                    longSenders[dat.seqNum] = std::unique_ptr<Sender>(new Sender(dat, cc, &sigMgr));
                    longSenders[dat.seqNum]->executeAsync();
                }
            }
            ros::Rate(LARGE_TX_FREQ).sleep();
        }
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
        {
            std::lock_guard<std::mutex> lock(mSmallTxQ);
            if (sendData.data.size() <= SIZE_THRESHOLD && smallTxQ.size() < MAX_QUEUE_SIZE)
            {
                smallTxQ.push(sendData);
                cvGotTx.notify_one();
                return true;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mLargeTxQ);
            if (sendData.data.size() > SIZE_THRESHOLD && largeTxQ.size() < MAX_QUEUE_SIZE)
            {
                largeTxQ.push(sendData);
                return true;
            }
        }
        return false;
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
        sigMgr.signal(WaitID(seqNum, segNum));
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
        largeTxThread = std::thread(&TxerTh::_runLarge, this);
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
        if (largeTxThread.joinable())
        {
            largeTxThread.join();
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