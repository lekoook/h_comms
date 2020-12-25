#ifndef H_REQS_MEDIATOR
#define H_REQS_MEDIATOR

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "requestors/Requestor.hpp"
#include "ADataAccessor.hpp"
#include "AReqManager.hpp"

/**
 * @brief A Mediator class that will spawn and manage all Requestors used to service data requests.
 * 
 */
class ReqsMediator : public AReqManager
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
    const int MAX_REQUESTORS = 10;

    /**
     * @brief The maximum number of times a particular request can be requeued.
     * 
     */
    const int MAX_REQUEST_ATTEMPTS = 3;

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
     * @brief Queue that tracks the sequence number of all Requestors that have ended it's life and marked for removal.
     * 
     */
    std::queue<uint32_t> reqToRemove;

    /**
     * @brief Mutex to protect the queue that marks Requestors for removal.
     * 
     */
    std::mutex mReqToRemove;

    /**
     * @brief Tracks the number of attempts a request sequence number has appeared.
     * 
     */
    std::unordered_multiset<uint32_t> reqTries;

    /**
     * @brief Mutex to protect the record for request sequence number appearances.
     * 
     */
    std::mutex mReqTries;

    /**
     * @brief Executes the processing of items in the requests queue.
     * 
     */
    void _runReq()
    {
        while(reqRunning.load())
        {
            // Remove all Requestors marked for removal.
            {
                std::lock_guard<std::mutex> rLock(mReqToRemove);
                while (reqToRemove.size() > 0)
                {
                    uint32_t key = reqToRemove.front();
                    reqToRemove.pop();
                    std::lock_guard<std::mutex> qLock(mReqRecord);
                    reqRecord.erase(key);
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
                                                transmitter, dataAccessor, this));
                }
            }

            ros::Duration(0.1).sleep();
        }
    }

    /**
     * @brief Internal method to submit a new request for data exchange.
     * 
     * @param sequence Sequence number for this request.
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     */
    void _queueReq(uint32_t sequence, uint16_t entryId, std::string reqTarget)
    {
        std::lock_guard<std::mutex> lock(mReqQ);
        reqQ.push(ReqQueueData(sequence, entryId, reqTarget));
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

    /**
     * @brief Mark a Requestor by their sequence number for removal from tracking.
     * 
     * @param sequence Sequence number of Requestor to remove.
     */
    void removeReq(uint32_t sequence)
    {
        std::lock_guard<std::mutex> lock(mReqToRemove);
        reqToRemove.push(sequence);
    }

    /**
     * @brief Requeues a request that was queued previously.
     * 
     * @param sequence Sequence number that was assigned to this request initially.
     * @param entryId Entry ID in the MIT to request.
     * @param reqTarget Target robot this request is intended for.
     */
    void requeueReq(uint32_t sequence, uint16_t entryId, std::string reqTarget)
    {
        std::lock_guard<std::mutex> lock(mReqTries);
        size_t counts = reqTries.count(sequence);
        if (counts < MAX_REQUEST_ATTEMPTS - 1)
        {
            reqTries.insert(sequence);
            _queueReq(sequence, entryId, reqTarget);
        }
        else
        {
            reqTries.erase(sequence);
        }
    }
};

#endif // H_REQS_MEDIATOR