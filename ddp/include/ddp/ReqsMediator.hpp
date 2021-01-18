#ifndef H_REQS_MEDIATOR
#define H_REQS_MEDIATOR

#include "ros/ros.h"
#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "requestors/Requestor.hpp"
#include "ADataAccessor.hpp"

/**
 * @brief A Mediator class that will spawn and manage all Requestors used to service data requests.
 * 
 */
class ReqsMediator
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
         * @brief Sequence number for this request.
         * 
         */
        uint32_t sequence;

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
         * @param sequence Sequence number for this request.
         * @param entryId Entry ID to request for.
         * @param target Intended target robot this request is for.
         */
        ReqQueueData(uint32_t sequence, uint16_t entryId, std::string target) : sequence(sequence), entryId(entryId), target(target) {}
    };

    /**
     * @brief Maximum number of concurrent Requestors allowed.
     * 
     */
    const int MAX_REQUESTORS = 2;

    /**
     * @brief The rate at which the new requests submission queue is serviced and handled.
     * 
     */
    const double QUEUE_RATE = 10.0f;

    /**
     * @brief Current sequence number used to generate each Request queue item.
     * 
     */
    uint32_t sequence = 0;

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
     * @brief Tracks the current entry IDs and target address in the request queue to prevent duplicated requests.
     * 
     */
    std::set<std::pair<std::string, uint16_t>> dupReq;

    /**
     * @brief Mutext to protect the requests queue.
     * 
     */
    std::mutex mReqQ;

    /**
     * @brief Interface used to transmit messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Interface used to access data in database.
     * 
     */
    ADataAccessor* dataAccessor;

    /**
     * @brief A record to track all concurrent Requestors running at once.
     * @details The key is the sequence number for the Requestor and value is the Requestor instance.
     * 
     */
    std::unordered_map<uint32_t, Requestor> reqRecord;

    /**
     * @brief Mutex to protect the Requestors record.
     * 
     */
    std::mutex mReqRecord;

    /**
     * @brief Executes the processing of items in the requests queue.
     * 
     */
    void _runReq()
    {
        while(reqRunning.load())
        {
            // Remove all Requestors that has ended it's life and restart those that needs to.
            {
                std::lock_guard<std::mutex> qLock(mReqRecord);
                for (auto it = reqRecord.begin(); it != reqRecord.end();)
                {
                    if (it->second.needRelive() && it->second.hasEnded()) // Only restart if it has fully ended.
                    {
                        it->second.start();
                    }
                    else if (it->second.hasEnded())
                    {
                        std::lock_guard<std::mutex> lock(mReqQ);
                        dupReq.erase(std::make_pair(it->second.getTarget(), it->second.getEntryId()));
                        it = reqRecord.erase(it);
                    }
                    else
                    {
                        it++;
                    }
                }
            }

            // We spawn as many Requestors as we can to service data requests.
            {
                std::lock_guard<std::mutex> qLock(mReqRecord);
                std::lock_guard<std::mutex> lock(mReqQ);
                while ((reqRecord.size() < MAX_REQUESTORS) && (!reqQ.empty()))
                {
                    ReqQueueData qData = reqQ.front();
                    reqQ.pop();
                    reqRecord.emplace(
                        std::piecewise_construct, 
                        std::forward_as_tuple(qData.sequence), 
                        std::forward_as_tuple(qData.sequence, qData.entryId, qData.target, 
                                                transmitter, dataAccessor));
                    ROS_INFO("Request sequence %u for entry %u to %s", 
                        qData.sequence, qData.entryId, qData.target.c_str());
                }
            }

            ros::Rate(QUEUE_RATE).sleep();
        }
    }

    /**
     * @brief Internal method to submit a new request for data exchange.
     * 
     * @param sequence Sequence number for this request.
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     * @param requeue Indicates if this is meant to be a requeue of a previously queued entry.
     */
    void _queueReq(uint32_t sequence, uint16_t entryId, std::string reqTarget, bool requeue=false)
    {
        std::lock_guard<std::mutex> lock(mReqQ);
        auto key = std::make_pair(reqTarget, entryId);
        // Prevent duplicate Requests from being queued if there already exists one in queue or is currently executing.
        if (requeue || (dupReq.find(key) == dupReq.end()))
        {
            dupReq.insert(key);
            reqQ.push(ReqQueueData(sequence, entryId, reqTarget));
        }
    }
    
public:
    /**
     * @brief Construct a new Reqs Mediator object that will handle one or more Requestors to conduct data requests.
     * 
     * @param transmitter Transmitter used to send messages.
     * @param dataAccessor Interface used to access data from the database.
     */
    ReqsMediator(ATransmitter* transmitter, ADataAccessor* dataAccessor) 
        : transmitter(transmitter), dataAccessor(dataAccessor)
    {
        reqRunning.store(true);
        reqTh = std::thread(&ReqsMediator::_runReq, this);
    }

    /**
     * @brief Destroy the Reqs Mediator object.
     * 
     */
    ~ReqsMediator()
    {
        stop();
    }

    /**
     * @brief ReqsMediator is not CopyConstructible.
     * 
     */
    ReqsMediator(const ReqsMediator& other) = delete;

    /**
     * @brief ReqsMediator is not CopyAssignable.
     * 
     */
    ReqsMediator& operator=(const ReqsMediator& other) = delete;

    /**
     * @brief ReqsMediator is not MoveConstructible.
     * 
     */
    ReqsMediator(ReqsMediator&& other) = delete;

    /**
     * @brief ReqsMediator is not MoveAssignable.
     * 
     */
    ReqsMediator& operator=(const ReqsMediator&& other) = delete;
    
    /**
     * @brief Stops the operation of handling requests.
     * 
     */
    void stop()
    {
        reqRunning.store(false);
        if (reqTh.joinable())
        {
            reqTh.join();
        }
    }

    /**
     * @brief Notify the intended Requestor that a ACK message has arrived.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    void notifyAck(std::string src, AckMsg& ackMsg)
    {
        std::lock_guard<std::mutex> lock(mReqRecord);
        auto it = reqRecord.find(ackMsg.ackSequence);
        if (it != reqRecord.end())
        {
            it->second.recvAck(ackMsg, src);
        }
    }

    /**
     * @brief Notify the intended Requestor that a DATA message has arrived.
     * 
     * @param src Source address of DATA message.
     * @param dataMsg DATA message.
     */
    void notifyData(std::string src, DataMsg& dataMsg)
    {
        std::lock_guard<std::mutex> lock(mReqRecord);
        auto it = reqRecord.find(dataMsg.reqSequence);
        if (it != reqRecord.end())
        {
            it->second.recvData(dataMsg, src);
        }
    }

    /**
     * @brief Submit a new request for data exchange.
     * 
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     */
    void queueReq(uint16_t entryId, std::string reqTarget)
    {
        _queueReq(sequence++, entryId, reqTarget);
    }
};

#endif // H_REQS_MEDIATOR