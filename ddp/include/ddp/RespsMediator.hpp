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

/**
 * @brief A Mediator class that will spawn and manage all the Responders used to service data requests.
 * 
 */
class RespsMediator
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
    const int MAX_RESPONDERS = 6;

    /**
     * @brief The rate at which the new responses submission queue is serviced and handled.
     * 
     */
    const double QUEUE_RATE = 10.0f;

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
     * @brief Executes the processing of items in the responses queue.
     * 
     */
    void _runReq()
    {
        while(respRunning.load())
        {
            // Remove all Responders that has ended it's life.
            {
                std::lock_guard<std::mutex> qLock(mRespRecord);
                for (auto it = respRecord.begin(); it != respRecord.end();)
                {
                    if (it->second.hasEnded())
                    {
                        it = respRecord.erase(it);
                    }
                    else
                    {
                        it++;
                    }
                }
            }

            // We spawn as many Responders as we can to service data requests.
            {
                std::lock_guard<std::mutex> qLock(mRespRecord);
                std::lock_guard<std::mutex> lock(mRespQ);
                while ((respRecord.size() < MAX_RESPONDERS) && (!respQ.empty()))
                {
                    RespQueueData qData = respQ.front();

                    auto key = std::make_pair(qData.target, qData.sequence);
                    if (respRecord.find(key) == respRecord.end())
                    {
                        respRecord.emplace(
                            std::piecewise_construct, 
                            std::forward_as_tuple(key), 
                            std::forward_as_tuple(qData.target, qData.sequence, qData.entryId, qData.target, 
                                                    transmitter, dataAccessor));
                        ROS_INFO("Response to sequence %u for entry %u from %s", 
                            qData.sequence, qData.entryId, qData.target.c_str());
                    }
                    respQ.pop();
                }
            }

            ros::Rate(QUEUE_RATE).sleep();
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
};

#endif // H_RESPS_MEDIATOR