#ifndef H_RESPS_MEDIATOR
#define H_RESPS_MEDIATOR

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include <map>
#include "responders/Responder.hpp"
#include "ADataAccessor.hpp"
#include "ARespManager.hpp"

class RespsMediator : public ARespManager
{
private:
    /**
     * @brief Represents an item in the response queue.
     * 
     */
    class RespQueueData
    {
    public:
        /**
         * @brief Sequence number for this response.
         * 
         */
        uint32_t sequence;

        /**
         * @brief Entry ID to respond for.
         * 
         */
        uint16_t entryId;

        /**
         * @brief Intended target robot this response is for.
         * 
         */
        std::string target;

        /**
         * @brief Construct a new Resp Queue Data object.
         * 
         */
        RespQueueData() {}

        /**
         * @brief Construct a new Resp Queue Data object.
         * 
         * @param sequence Sequence number for this response.
         * @param entryId Entry ID to respond for.
         * @param target Intended target robot this response is for.
         */
        RespQueueData(uint32_t sequence, uint16_t entryId, std::string target) 
            : sequence(sequence), entryId(entryId), target(target) {}
    };

    /**
     * @brief Maximum number of concurrent Requestors allowed.
     * 
     */
    const int MAX_RESPONDERS = 10;

    /**
     * @brief Response thread to handle new responses submissions.
     * 
     */
    std::thread respTh;

    /**
     * @brief Flag to control the execution of response thread.
     * 
     */
    std::atomic<bool> respRunning;

    /**
     * @brief First-In-First-Out queue to store new responses submissions.
     * 
     */
    std::queue<RespQueueData> respQ;

    /**
     * @brief Mutext to protect the responses queue.
     * 
     */
    std::mutex mRespQ;

    /**
     * @brief Interface used to transmit messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Interface used to access data from the database.
     * 
     */
    ADataAccessor* dataAccessor;

    /**
     * @brief A record to track all concurrent Responders running at once.
     * @details The key is a pair with the first being the source address of Requestor and second being the sequence 
     * number of the request. The value is the Responder instance.
     * 
     */
    std::map<std::pair<std::string, uint32_t>, Responder> respRecord;

    /**
     * @brief Mutex to protect the Responders record.
     * 
     */
    std::mutex mRespRecord;

    /**
     * @brief Queue that tracks the Requestor address and sequence number of all Responder that have ended it's life 
     * and marked for removal.
     * 
     */
    std::queue<std::pair<std::string, uint32_t>> respToRemove;

    /**
     * @brief Mutex to protect the queue that marks Responders for removal.
     * 
     */
    std::mutex mRespToRemove;

    /**
     * @brief Executes the processing of items in the responses queue.
     * 
     */
    void _runReq()
    {
        while(respRunning.load())
        {
            // Remove all Responders marked for removal.
            {
                std::lock_guard<std::mutex> rLock(mRespToRemove);
                while (respToRemove.size() > 0)
                {
                    auto key = respToRemove.front();
                    respToRemove.pop();
                    std::lock_guard<std::mutex> qLock(mRespRecord);
                    respRecord.erase(key);
                }
            }

            // We spawn as many Responders as we can to service data requests.
            {
                std::lock_guard<std::mutex> qLock(mRespRecord);
                std::lock_guard<std::mutex> lock(mRespQ);
                while ((respRecord.size() < MAX_RESPONDERS) && (!respQ.empty()))
                {
                    RespQueueData qData = respQ.front();
                    respQ.pop();
                    respRecord.emplace(
                        std::piecewise_construct, 
                        std::forward_as_tuple(std::make_pair(qData.target, qData.sequence)), 
                        std::forward_as_tuple(qData.target, qData.sequence, qData.entryId, qData.target, 
                                                transmitter, dataAccessor, this));
                }
            }

            ros::Duration(0.1).sleep();
        }
    }
    
public:
    /**
     * @brief Construct a new Resps Mediator object.
     * 
     * @param transmitter Transmitter used to send messages.
     * @param dataAccessor Interface used to access data from the database.
     */
    RespsMediator(ATransmitter* transmitter, ADataAccessor* dataAccessor) 
        : transmitter(transmitter), dataAccessor(dataAccessor)
    {
        respRunning.store(true);
        respTh = std::thread(&RespsMediator::_runReq, this);
    }

    /**
     * @brief Destroy the Resps Mediator object.
     * 
     */
    ~RespsMediator()
    {
        stop();
    }

    /**
     * @brief RespsMediator is not CopyConstructible.
     * 
     */
    RespsMediator(const RespsMediator& other) = delete;

    /**
     * @brief RespsMediator is not CopyAssignable.
     * 
     */
    RespsMediator& operator=(const RespsMediator& other) = delete;

    /**
     * @brief RespsMediator is not MoveConstructible.
     * 
     */
    RespsMediator(const RespsMediator&& other) = delete;

    /**
     * @brief RespsMediator is not MoveAssignable.
     * 
     */
    RespsMediator& operator=(const RespsMediator&& other) = delete;

    /**
     * @brief Stops the operation of handling requests.
     * 
     */
    void stop()
    {
        respRunning.store(false);
        if (respTh.joinable())
        {
            respTh.join();
        }
    }

    /**
     * @brief Notify that a ACK message has arrived.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    void notifyAck(std::string src, AckMsg& ackMsg)
    {
        std::lock_guard<std::mutex> lock(mRespRecord);
        auto it = respRecord.find(std::make_pair(src, ackMsg.ackSequence));
        if (it != respRecord.end())
        {
            it->second.recvAck(ackMsg, src);
        }
    }

    /**
     * @brief Submit a new response for data exchange.
     * 
     * @param sequence Sequence number of the initial request.
     * @param entryId Entry ID in the MIT to respond for.
     * @param reqTarget Target robot this response is intended for.
     */
    void queueResp(uint32_t sequence, uint16_t entryId, std::string respTarget)
    {
        std::lock_guard<std::mutex> lock(mRespQ);
        respQ.push(RespQueueData(sequence, entryId, respTarget));
    }

    /**
     * @brief Mark a Responder for removal from tracking.
     * 
     * @param src Source address of request to remove.
     * @param sequence Sequence number of request to remove.
     */
    void removeResp(std::string src, uint32_t sequence)
    {
        std::lock_guard<std::mutex> lock(mRespToRemove);
        respToRemove.push(std::make_pair(src, sequence));
    }
};

#endif // H_RESPS_MEDIATOR