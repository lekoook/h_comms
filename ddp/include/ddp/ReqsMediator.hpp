#ifndef H_REQS_MEDIATOR
#define H_REQS_MEDIATOR

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <queue>
#include "requestors/Requestor.hpp"

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
     * @brief Pointer to the current Requestor. If there is no Requestor, this should be null pointer.
     * 
     */
    Requestor* requestor;

    /**
     * @brief Mutex to protect the Requestor.
     * 
     */
    std::mutex mRequestor;

    /**
     * @brief Executes the processing of items in the requests queue.
     * 
     */
    void _runReq()
    {
        while(reqRunning.load())
        {
            ros::Duration(0.1).sleep();

            bool available = false;
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
                Requestor req(qData.sequence, qData.entryId, qData.target, transmitter);

                {
                    std::lock_guard<std::mutex> lock(mRequestor);
                    requestor = &req;
                }
                
                while (!req.hasEnded()) {}

                {
                    std::lock_guard<std::mutex> lock(mRequestor);
                    requestor = nullptr;
                }
            }
        }
    }
    
public:
    /**
     * @brief Construct a new Reqs Mediator object that will handle one or more Requestors to conduct data requests.
     * 
     * @param transmitter Transmitter used to send messages.
     */
    ReqsMediator(ATransmitter* transmitter) : transmitter(transmitter)
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
     * @brief Notify that a ACK message has arrived.
     * 
     * @param src Source address of ACK message.
     * @param ackMsg Acknowledgement (ACK) message.
     */
    void notifyAck(std::string src, AckMsg& ackMsg)
    {
        std::lock_guard<std::mutex> lock(mRequestor);
        if (requestor)
        {
            requestor->recvAck(ackMsg, src);
        }
    }

    /**
     * @brief Notify that a DATA message has arrived.
     * 
     * @param src Source address of DATA message.
     * @param dataMsg DATA message.
     */
    void notifyData(std::string src, DataMsg& dataMsg)
    {
        std::lock_guard<std::mutex> lock(mRequestor);
        if (requestor)
        {
            requestor->recvData(dataMsg, src);
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
        std::lock_guard<std::mutex> lock(mReqQ);
        reqQ.push(ReqQueueData(sequence++, entryId, reqTarget));
    }
};

#endif // H_REQS_MEDIATOR