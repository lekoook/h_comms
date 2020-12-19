#ifndef H_TXER_TH
#define H_TXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include "subt_communication_broker/subt_communication_client.h"
#include "Packet.hpp"


class TxerTh
{
private:
    class TxQueueData
    {
    public:
        int8_t tries = 0;
        uint32_t seqNum = 0;
        std::vector<uint8_t> data;
        std::string dest;
        TxQueueData () {}
        TxQueueData (std::vector<uint8_t> data, std::string dest, uint32_t seqNum) 
            : data(data), dest(dest), seqNum(seqNum)
        {}
    };

    // Constants
    uint8_t const MAX_QUEUE_SIZE = 10;
    uint8_t const MAX_QUEUE_TRIES = 3;
    uint8_t const MAX_SEGMENT_TRIES = 3;

    std::thread txThread;
    std::atomic<bool> thRunning;
    std::queue<TxQueueData> txQ;
    std::mutex mTxQ;
    subt::CommsClient* cc;
    std::mutex mAck;
    std::condition_variable cvAck;
    bool gotAck;
    std::mutex mWaitAck;
    std::string waitSrc;
    uint32_t waitSeq;
    uint8_t waitSeg;
    uint32_t sequence = 0;

    void _run()
    {
        while(thRunning.load())
        {
            bool empty = false;
            {
                std::lock_guard<std::mutex> lock(mTxQ);
                empty = txQ.empty();
            }
            if (!empty)
            {
                std::lock_guard<std::mutex> lock(mTxQ);
                TxQueueData dat = txQ.front();
                txQ.pop();

                // Only send if the destination is a neighbour.
                subt::CommsClient::Neighbor_M nb = cc->Neighbors();
                if (nb.find(dat.dest) == nb.end() || !sendData(dat))
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

    bool sendData(const TxQueueData& data)
    {
        int dSize = data.data.size();
        int segs = (dSize / Packet::MAX_SEGMENT_SIZE) + ((dSize % Packet::MAX_SEGMENT_SIZE) != 0);
        uint8_t* ptr = (uint8_t*)data.data.data();
        uint16_t s = 0;

        for (size_t i = 0; i < dSize; i+= Packet::MAX_SEGMENT_SIZE)
        {
            int tries = 0;
            auto last = std::min((unsigned long)dSize, i + Packet::MAX_SEGMENT_SIZE);
            do
            {
                cc->SendTo(
                    Packet(data.seqNum, s, dSize, std::vector<uint8_t>(ptr + i, ptr + last)).serialize(), data.dest);
                tries++;
            } while (!_waitAck(data.seqNum, s, data.dest, 1000) && tries < MAX_SEGMENT_TRIES);

            if (tries >= MAX_SEGMENT_TRIES)
            {
                return false;
            }
            s++;
        }
        return true;
    }

    bool _queueOne(TxQueueData& sendData)
    {
        if (txQ.size() < MAX_QUEUE_SIZE)
        {
            txQ.push(sendData);
            return true;
        }
        else
        {
            return false;
        }
    }

    bool _waitAck(uint32_t seqNum, uint8_t segNum, std::string src, uint32_t timeout)
    {
        {
            std::lock_guard<std::mutex> wLock(mWaitAck);
            waitSrc = src;
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

    void _signalAck(uint32_t seqNum, uint8_t segNum, std::string src)
    {
        uint32_t tseq;
        uint8_t tseg;
        std::string tsrc;
        {
            std::lock_guard<std::mutex> wLock(mWaitAck);
            tseq = waitSeq;
            tseg = waitSeg;
            tsrc = waitSrc;
        }

        if (seqNum == tseq && segNum == tseg && src == tsrc)
        {
            std::lock_guard<std::mutex> lock(mAck);
            gotAck = true;
            cvAck.notify_one(); // Inform that an ACK has been received.
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
    TxerTh(subt::CommsClient* cc) : cc(cc)
    {
        thRunning.store(false);
    }
    
    ~TxerTh()
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
        txThread = std::thread(&TxerTh::_run, this);
    }

    void stop()
    {
        thRunning.store(false);
        if (txThread.joinable())
        {
            txThread.join();
        }
    }

    bool sendOne(std::vector<uint8_t> data, std::string dest)
    {
        std::lock_guard<std::mutex> lock(mTxQ);
        TxQueueData sendData(data, dest, sequence++);
        return _queueOne(sendData);
    }

    void notifyAck(uint32_t seqNum, uint8_t segNum, std::string src)
    {
        _signalAck(seqNum, segNum, src);
    }
};

#endif // H_TXER_TH