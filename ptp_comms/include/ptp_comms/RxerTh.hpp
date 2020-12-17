#ifndef H_RXER_TH
#define H_RXER_TH

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include "subt_communication_broker/subt_communication_client.h"

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

    std::string _uint8_to_string(std::vector<uint8_t>& b)
    {
        return std::string((char *)b.data(), b.size());
    }

    std::vector<uint8_t> _string_to_uint8(std::string& s)
    {
        return std::vector<uint8_t>(s.begin(), s.end());
    }
    
public:
    RxerTh(subt::CommsClient* cc) : cc(cc)
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