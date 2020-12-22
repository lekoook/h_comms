#ifndef H_RESPS_MEDIATOR
#define H_RESPS_MEDIATOR

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include "ARespHandler.hpp"
#include "responders/Responder.hpp"

class RespsMediator : public ARespHandler
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
     * @brief Pointer to the current Responder. If there is no Responder, this should be null pointer.
     * 
     */
    Responder* responder;

    /**
     * @brief Mutex to protect the Responder.
     * 
     */
    std::mutex mResponder;

    /**
     * @brief Executes the processing of items in the responses queue.
     * 
     */
    void _runReq()
    {
        while(respRunning.load())
        {
            ros::Duration(0.1).sleep();

            bool available = false;
            RespQueueData qData;
            {
                std::lock_guard<std::mutex> lock(mRespQ);
                if (!respQ.empty())
                {
                    available = true;
                    qData = respQ.front();
                    respQ.pop();
                }
            }

            if (available)
            {
                Responder resp(qData.sequence, qData.entryId, qData.target, transmitter);

                {
                    std::lock_guard<std::mutex> lock(mResponder);
                    responder = &resp;
                }

                while (!resp.hasEnded()) {}

                {
                    std::lock_guard<std::mutex> lock(mResponder);
                    responder = nullptr;
                }
            }
        }
    }
    
public:
    /**
     * @brief Construct a new Resps Mediator object.
     * 
     */
    RespsMediator()
    {
        transmitter = nullptr;
        responder = nullptr;
        respRunning.store(false);
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
     * @brief Starts the operation of handling responses.
     * 
     * @param transmitter Interface used to transmit messages when handling responses.
     */
    void start(ATransmitter* transmitter)
    {
        if (respRunning.load())
        {
            return; // Don't start operations twice.
        }
        
        this->transmitter = transmitter;
        respRunning.store(true);
        respTh = std::thread(&RespsMediator::_runReq, this);
    }

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
        std::lock_guard<std::mutex> lock(mResponder);
        if (responder)
        {
            responder->recvAck(ackMsg, src);
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