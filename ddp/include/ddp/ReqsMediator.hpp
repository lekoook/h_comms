#ifndef H_REQS_MEDIATOR
#define H_REQS_MEDIATOR

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include "AReqHandler.hpp"

class ReqsMediator : public AReqHandler
{
private:
    /**
     * @brief Represents an item in the request queue.
     * 
     */
    class ReqQueueData
    {
    public:
        /**
         * @brief Entry ID to request for.
         * 
         */
        uint16_t entryId;

        /**
         * @brief Intended target robot this request is for.
         * 
         */
        std::string target;

        /**
         * @brief Construct a new Req Queue Data object.
         * 
         */
        ReqQueueData() {}

        /**
         * @brief Construct a new Req Queue Data object.
         * 
         * @param entryId Entry ID to request for.
         * @param target Intended target robot this request is for.
         */
        ReqQueueData(uint16_t entryId, std::string target) : entryId(entryId), target(target) {}
    };

    /**
     * @brief Request thread to handle new requests submissions.
     * 
     */
    std::thread reqTh;

    /**
     * @brief Flag to control the execution of request thread.
     * 
     */
    std::atomic<bool> reqRunning;

    /**
     * @brief First-In-First-Out queue to store new requests submissions.
     * 
     */
    std::queue<ReqQueueData> reqQ;

    /**
     * @brief Mutext to protect the requests queue.
     * 
     */
    std::mutex mReqQ;

    /**
     * @brief Executes the processing of items in the requests queue.
     * 
     */
    void _runReq()
    {
        while(reqRunning.load())
        {
            bool available;
            ReqQueueData qData;
            {
                std::lock_guard<std::mutex> lock(mReqQ);
                if (!reqQ.empty())
                {
                    available = true;
                    qData = reqQ.front();
                    reqQ.pop();
                }
            }

            if (available)
            {
                available = false;
                // Make a new request.

                ROS_INFO("%s", qData.target.c_str());
                ROS_INFO("%u", qData.entryId);
            }
        }
    }
    
public:
    ReqsMediator() 
    {
        reqRunning.store(true);
        reqTh = std::thread(&ReqsMediator::_runReq, this);
    }

    /**
     * @brief Notify that a ACK message has arrived.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    void notifyAck(std::string src, AckMsg& ackMsg)
    {
        
    }

    /**
     * @brief Notify that a DATA message has arrived.
     * 
     * @param src Source address of DATA message.
     * @param dataMsg DATA message.
     */
    void notifyData(std::string src, DataMsg& dataMsg)
    {

    }

    /**
     * @brief Submit a new request for data exchange.
     * 
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     */
    void queueReq(uint16_t entryId, std::string reqTarget)
    {
        std::lock_guard<std::mutex> lock(mReqQ);
        reqQ.push(ReqQueueData(entryId, reqTarget));
    }
};

#endif // H_REQS_MEDIATOR