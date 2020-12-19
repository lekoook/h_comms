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

class RxQueueData
{
public:
    std::string data;
    std::string src;
    RxQueueData () {}
    RxQueueData (std::string data, std::string src) : data(data), src(src)
    {}
};

class RxerTh
{
private:
    // Constants
    uint8_t const MAX_QUEUE_SIZE = 10;
    uint8_t const MAX_QUEUE_TRIES = 3;
    uint32_t const MAX_ENTRY_AGE = 300; // Seconds
    int32_t const CLEAN_SLEEP_TIME = 30; // Seconds

    std::thread rxThread;
    std::atomic<bool> thRunning;
    std::queue<RxQueueData> rxQ;
    std::mutex mRxQ;
    subt::CommsClient* cc;
    TxerTh* txerTh;
    SegTable segTable;
    std::function<void(std::string, std::vector<uint8_t>&)> rxCb;
    std::map<std::tuple<std::string, uint32_t, uint8_t>, uint32_t> seenRx;
    std::mutex mSeenRx;
    std::thread cleanSeenRxTh;
    std::atomic<bool> cleanRunning;

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

    void _ack(std::string dest, Packet& received)
    {
        Packet pkt(received.seqNum, received.segNum, 0, std::vector<uint8_t>(), true);
        std::string ser = pkt.serialize();
        cc->SendTo(ser, dest);
    }

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

    void _cleanSeenRx()
    {
        while(cleanRunning.load())
        {
            {
                std::lock_guard<std::mutex> lock(mSeenRx);
                for (auto entry : seenRx)
                {
                    std::cout << std::get<1>(entry.first) << " : " << entry.second << std::endl;
                }
                
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

    std::string _uint8_to_string(std::vector<uint8_t>& b)
    {
        return std::string((char *)b.data(), b.size());
    }

    std::vector<uint8_t> _string_to_uint8(std::string& s)
    {
        return std::vector<uint8_t>(s.begin(), s.end());
    }
    
public:
    RxerTh(subt::CommsClient* cc, TxerTh* txerTh, std::function<void(std::string, std::vector<uint8_t>&)> rxCb) : cc(cc), txerTh(txerTh), rxCb(rxCb)
    {
        thRunning.store(false);
    }
    
    ~RxerTh()
    {
        stop();
    }
    
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

    bool recvOne(RxQueueData recvData)
    {
        std::lock_guard<std::mutex> lock(mRxQ);
        return _recvOne(recvData);
    }
};

#endif // H_RXER_TH