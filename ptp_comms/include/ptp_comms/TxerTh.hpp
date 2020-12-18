#ifndef H_TXER_TH
#define H_TXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include "subt_communication_broker/subt_communication_client.h"
#include "Packet.hpp"

class TxQueueData
{
public:
    int8_t tries = 0;
    std::vector<uint8_t> data;
    std::string dest;
    TxQueueData () {}
    TxQueueData (std::vector<uint8_t> data, std::string dest) : data(data), dest(dest)
    {}
};

class TxerTh
{
private:
    // Constants
    uint8_t const MAX_QUEUE_SIZE = 10;
    uint8_t const MAX_QUEUE_TRIES = 3;
    uint32_t const MAX_PACKET_SIZE = 10; // Constrained by subt API
    uint32_t const MAX_SEGMENT_SIZE = MAX_PACKET_SIZE - Packet::FIXED_LEN;

    std::thread txThread;
    std::atomic<bool> thRunning;
    std::queue<TxQueueData> txQ;
    std::mutex mTxQ;
    subt::CommsClient* cc;

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
                if (nb.find(dat.dest) != nb.end())
                {
                    sendData(dat.data, dat.dest);
                }
                else
                {
                    dat.tries++;
                    if (dat.tries < MAX_QUEUE_TRIES)
                    {
                        _sendOne(dat); // Re-queue the data back into the queue to be processed again later. Give up after MAX_QUEUE_TRIES attempts.
                    }
                }
            }
        }
    }

    void sendData(const std::vector<uint8_t>& data, std::string dest)
    {
        int dSize = data.size();
        int segs = (dSize / MAX_SEGMENT_SIZE) + ((dSize % MAX_SEGMENT_SIZE) != 0);
        uint8_t* ptr = (uint8_t*)data.data();
        uint16_t s = 0;

        for (size_t i = 0; i < dSize; i+= MAX_SEGMENT_SIZE)
        {
            auto last = std::min((unsigned long)dSize, i + MAX_SEGMENT_SIZE);
            cc->SendTo(
                Packet(sequence, (uint8_t)segs, s++, std::vector<uint8_t>(ptr + i, ptr + last)).serialize(), dest);
        }
        sequence++;
    }

    bool _sendOne(TxQueueData& sendData)
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

    bool sendOne(TxQueueData sendData)
    {
        std::lock_guard<std::mutex> lock(mTxQ);
        return _sendOne(sendData);
    }
};

#endif // H_TXER_TH