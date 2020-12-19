#ifndef H_RXER_TH
#define H_RXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include <functional>
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

    std::thread rxThread;
    std::atomic<bool> thRunning;
    std::queue<RxQueueData> rxQ;
    std::mutex mRxQ;
    subt::CommsClient* cc;
    TxerTh* txerTh;
    SegTable segTable;
    std::function<void(std::string, std::vector<uint8_t>&)> rxCb;

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
                    // _ack(dat.src, pkt);
                    if (segTable.updateSeg(dat.src, pkt))
                    {
                        std::vector<uint8_t> v = segTable.getFullData(dat.src, pkt.seqNum);
                        rxCb(dat.src, v);
                    }
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
    }

    void stop()
    {
        thRunning.store(false);
        if (rxThread.joinable())
        {
            rxThread.join();
        }
    }

    bool recvOne(RxQueueData recvData)
    {
        std::lock_guard<std::mutex> lock(mRxQ);
        return _recvOne(recvData);
    }
};

#endif // H_RXER_TH