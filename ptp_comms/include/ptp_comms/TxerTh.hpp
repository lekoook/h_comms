#ifndef H_TXER_TH
#define H_TXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include "subt_communication_broker/subt_communication_client.h"

class TxQueueData
{
public:
    int8_t tries = 0;
    std::vector<uint8_t> data;
    std::string dest;
    TxQueueData () {}
    TxQueueData (std::vector<uint8_t> data, std::string dest)
    {
        this->data = data;
        this->dest = dest;
    }
};

class TxerTh
{
private:
    // Constants
    uint8_t const MAX_QUEUE_SIZE = 10;
    uint8_t const MAX_QUEUE_TRIES = 3;

    std::thread txThread;
    std::atomic<bool> thRunning;
    std::queue<TxQueueData> txQ;
    std::mutex mTxQ;
    subt::CommsClient* cc;

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
                    cc->SendTo(_uint8_to_string(dat.data), dat.dest);
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