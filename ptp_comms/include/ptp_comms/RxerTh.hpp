#ifndef H_RXER_TH
#define H_RXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include <functional>
#include <map>
#include "subt_communication_broker/subt_communication_client.h"
#include "SegTable.hpp"

/**
 * @brief Represents an item in the queue for received data.
 * 
 */
class RxQueueData
{
public:
    /**
     * @brief Actual payload data received.
     * 
     */
    std::string data;

    /**
     * @brief Source address of this data.
     * 
     */
    std::string src;

    /**
     * @brief Construct a new Rx Queue Data object.
     * 
     */
    RxQueueData () {}

    /**
     * @brief Construct a new Rx Queue Data object.
     * 
     * @param data Actual payload data received.
     * @param src Source address of this data.
     */
    RxQueueData (std::string data, std::string src) : data(data), src(src)
    {}
};

/**
 * @brief Represents a thread that processes data that has been received.
 * 
 */
class RxerTh
{
private:
    // Constants
    /**
     * @brief Maximum size of the reception queue
     * 
     */
    uint8_t const MAX_QUEUE_SIZE = 10;

    /**
     * @brief Maximum time (seconds) an entry in received tracking list is allowed to live.
     * 
     */
    uint32_t const MAX_ENTRY_AGE = 300; // Seconds

    /**
     * @brief Time (seconds) interval between each clean cycle of received tracking list.
     * 
     */
    int32_t const CLEAN_SLEEP_TIME = 30; // Seconds

    /**
     * @brief Reception thread that will run the actual data handling and ACK-ing.
     * 
     */
    std::thread rxThread;

    /**
     * @brief Flag to indicate if reception thread should run.
     * 
     */
    std::atomic<bool> thRunning;

    /**
     * @brief First-In-First-Out reception queue for storing pieces of data received.
     * 
     */
    std::queue<RxQueueData> rxQ;
    
    /**
     * @brief Mutex to protect the reception queue.
     * 
     */
    std::mutex mRxQ;

    /**
     * @brief subt::CommsClient that performs the actual transmission.
     * 
     */
    subt::CommsClient* cc;

    /**
     * @brief Transmission thread used to perform transmission. Used to notify it of ACK messages.
     * 
     */
    TxerTh* txerTh;

    /**
     * @brief Segment table that processes and tracks each received segment.
     * 
     */
    SegTable segTable;

    /**
     * @brief Callback function to call deliver a full data that has been reassembled from it's segments.
     * 
     */
    std::function<void(std::string, std::vector<uint8_t>&)> rxCb;

    /**
     * @brief Tracks a list of recently received packets.
     * @details The map key is a tuple where the first member is the source address, second member is the sequence 
     * number and the third member is the segment number. The map value is a timestamp given to this entry that
     * indicates how old this entry is. Entries that are too old are purged.
     * 
     */
    std::map<std::tuple<std::string, uint32_t, uint8_t>, uint32_t> seenRx;

    /**
     * @brief Mutex to protect the received packets tracking list.
     * 
     */
    std::mutex mSeenRx;

    /**
     * @brief Clearning thread that regularly cleans the received packets tracking list.
     * 
     */
    std::thread cleanSeenRxTh;

    /**
     * @brief Flag to indicate cleaning thread should run.
     * 
     */
    std::atomic<bool> cleanRunning;

    /**
     * @brief Executes the processing of data.
     * @details A piece of data is removed from the reception queue and handled according. If it is a data segment, it
     * be handled with the received packets tracking list and segment table. If it is an ACK message, it will used to 
     * notify the transmission thread of this ACK.
     * 
     */
    void _run()
    {
        while(thRunning.load())
        {
            bool empty = false;
            {
                std::lock_guard<std::mutex> lock(mRxQ);
                empty = rxQ.empty();
            }
            if (!empty)
            {
                std::lock_guard<std::mutex> lock(mRxQ);
                RxQueueData dat = rxQ.front();
                rxQ.pop();

                Packet pkt;
                pkt.deserialize(dat.data);

                // If this is not an ACK, we reply an ACK and process the segment.
                if (!pkt.isAck)
                {
                    _handleData(pkt, dat.src);
                }
                else
                {
                    txerTh->notifyAck(pkt.seqNum, pkt.segNum, dat.src); // Notify that an ACK has been received.
                }
            }
        }
    }

    /**
     * @brief Inserts one piece of data into the reception queue.
     * 
     * @param recvData Data received.
     * @return true If queue is not full and successful.
     * @return false If queue is already full.
     */
    bool _recvOne(RxQueueData& recvData)
    {
        if (rxQ.size() < MAX_QUEUE_SIZE)
        {
            rxQ.push(recvData);
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Transmits an acknowledgement (ACK) packet.
     * 
     * @param dest Destination of this ACK.
     * @param received Packet that this ACK is supposed to respond to.
     */
    void _ack(std::string dest, Packet& received)
    {
        Packet pkt(received.seqNum, received.segNum, 0, std::vector<uint8_t>(), true);
        std::string ser = pkt.serialize();
        cc->SendTo(ser, dest);
    }

    /**
     * @brief Handles a data segment.
     * @details This will first check if we have recently received the same segment. If yes, it will disregarded.
     * If no, it will be processed into the segment table accordingly. The received packets tracking list will also be updated accordingly.
     * 
     * @param packet Packet to process.
     * @param src Source address of the packet.
     */
    void _handleData(Packet& packet, std::string src)
    {
        _ack(src, packet); // ACK immediately first.

        // Find out if we have already seen this src, sequence, segment number recently.
        auto key = std::make_tuple(src, packet.seqNum, packet.segNum);
        bool notSeen = false;
        {
            std::lock_guard<std::mutex> lock(mSeenRx);
            notSeen = seenRx.find(key) == seenRx.end();
            seenRx[key] = ros::Time::now().sec; // Update the timestamp of this seen RX entry.
        }

        // If we have not seen this recently, process the data.
        if (notSeen)
        {
            if (segTable.updateSeg(src, packet))
            {
                std::vector<uint8_t> v = segTable.getFullData(src, packet.seqNum);
                rxCb(src, v);
            }
        }
    }

    /**
     * @brief Executes the cleaning of the received packets tracking list.
     * @details This will go through the list regularly and check which entry is older than \p MAX_ENTRY_AGE.
     * These entries will be removed.
     * 
     */
    void _cleanSeenRx()
    {
        while(cleanRunning.load())
        {
            {
                std::lock_guard<std::mutex> lock(mSeenRx);
                // Go through all entries and see which one is old enough to be removed.
                int64_t current = ros::Time::now().sec;
                for (auto it = seenRx.begin(); it != seenRx.end();)
                {
                    if (current - (int64_t)it->second > MAX_ENTRY_AGE)
                    {
                        it = seenRx.erase(it);
                    }
                    else
                    {
                        it++;
                    }
                }
            }
            ros::Duration(CLEAN_SLEEP_TIME, 0).sleep();
        }
    }

public:
    /**
     * @brief Construct a new Rxer Th object.
     * 
     * @param cc subt::CommsClient that performs the actual transmission.
     * @param txerTh Transmission thread used to notify it of acknowledgement packets.
     * @param rxCb Callback function to call deliver a full data that has been reassembled from it's segments.
     */
    RxerTh(subt::CommsClient* cc, TxerTh* txerTh, std::function<void(std::string, std::vector<uint8_t>&)> rxCb) : cc(cc), txerTh(txerTh), rxCb(rxCb)
    {
        thRunning.store(false);
    }
    
    /**
     * @brief Destroy the Rxer Th object.
     * 
     */
    ~RxerTh()
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
        rxThread = std::thread(&RxerTh::_run, this);
        cleanRunning.store(true);
        cleanSeenRxTh = std::thread(&RxerTh::_cleanSeenRx, this);
    }

    /**
     * @brief Stops the operation of this thread.
     * 
     */
    void stop()
    {
        thRunning.store(false);
        cleanRunning.store(false);
        if (rxThread.joinable())
        {
            rxThread.join();
        }
        if (cleanSeenRxTh.joinable())
        {
            cleanSeenRxTh.join();
        }
    }

    /**
     * @brief Queues the data that was just received.
     * 
     * @param recvData Data received.
     * @return true If queue is not full and successfully queued.
     * @return false If queue is already full.
     */
    bool recvOne(RxQueueData recvData)
    {
        std::lock_guard<std::mutex> lock(mRxQ);
        return _recvOne(recvData);
    }
};

#endif // H_RXER_TH