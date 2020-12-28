#ifndef H_DDP
#define H_DDP

#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <string>
#include "ptp_comms/PtpClient.hpp"
#include "messages/AdvMsg.hpp"
#include "ATransmitter.hpp"
#include "ADataAccessor.hpp"
#include "MIT.hpp"
#include "ReqsMediator.hpp"
#include "RespsMediator.hpp"

/**
 * @brief Encapsulates all data and logic for the workings of a Data Distribution Protocol (DDP).
 * 
 */
class DDP : private ATransmitter, private ADataAccessor
{
private:
    /**
     * @brief Represents an item in the message reception queue.
     * 
     */
    class RxQueueData
    {
    public:
        /**
         * @brief Actual payload data received.
         * 
         */
        std::vector<uint8_t> data;

        /**
         * @brief Source address of this data.
         * 
         */
        std::string src;

        /**
         * @brief Construct a new Rx Queue Data object.
         * 
         */
        RxQueueData () {}

        /**
         * @brief Construct a new Rx Queue Data object.
         * 
         * @param data Actual payload data received.
         * @param src Source address of this data.
         */
        RxQueueData (std::vector<uint8_t> data, std::string src) : data(data), src(src)
        {}
    };

    // Constants
    /**
     * @brief Time (seconds) between each advertisement message.
     * 
     */
    const double ADV_INTERVAL = 5.0;

    /**
     * @brief ptp_comms::PtpClient used to transmit and receive data.
     * 
     */
    std::unique_ptr<ptp_comms::PtpClient> ptpClient;

    /**
     * @brief Reception thread that processes all the received and queued messages.
     * 
     */
    std::thread rxHandleTh;

    /**
     * @brief Flag to control the execution of reception thread.
     * 
     */
    std::atomic<bool> mainRunning;

    /**
     * @brief First-In-First-Out queue to store all received messages.
     * 
     */
    std::queue<RxQueueData> rxQ;

    /**
     * @brief Mutex to protect the received messages queue.
     * 
     */
    std::mutex mRxQ;

    /**
     * @brief Condition variable to signal the main thread that new message has been received.
     * 
     */
    std::condition_variable cvRxQ;

    /**
     * @brief Advertisement thread that will broadcast the local robot's MIT regularly.
     * 
     */
    std::thread advTh;

    /**
     * @brief Flag to control the execution of advertisement thread.
     * 
     */
    std::atomic<bool> advRunning;

    /**
     * @brief Mediator for one or more Requestors.
     * 
     */
    std::unique_ptr<ReqsMediator> reqsMediator;

    /**
     * @brief Mediator for one or more Responders.
     * 
     */
    std::unique_ptr<RespsMediator> respsMediator;

    /**
     * @brief Callback to receive data.
     * 
     * @param src Source address of received data.
     * @param port Port of received data.
     * @param data Actual data received.
     */
    void _rxCb(std::string src, uint16_t port, std::vector<uint8_t> data)
    {
        std::lock_guard<std::mutex> lock(mRxQ);
        rxQ.push(RxQueueData(data, src));
        cvRxQ.notify_one();
    }

    /**
     * @brief Executes the processing and handling of received messages.
     * 
     */
    void _runMain()
    {
        // Main thread execution.
        while(mainRunning.load())
        {
            RxQueueData rxData;
            {
                std::unique_lock<std::mutex> lock(mRxQ);
                cvRxQ.wait(lock,
                    [this] () -> bool
                    {
                        return !rxQ.empty();
                    });
                rxData = rxQ.front();
                rxQ.pop();
            }

            _handleRxData(rxData.src, rxData.data);
        }
    }

    /**
     * @brief Executes the advertisement of local robot's MIT at a regular interval.
     * 
     */
    void _runAdv()
    {
        // Advertisement operation
        while(advRunning.load())
        {
            // TODO: Request MIT from ROS Service
            MIT mit;
            std::string ns = ros::this_node::getNamespace();
            if (ns == "/X1")
            {
                mit.update(1, 1, 1001);
                mit.update(1, 2, 1002);
                mit.update(1, 3, 1003);
                mit.update(1, 4, 1004);
                mit.update(1, 5, 1005);
            }
            else if (ns == "/X2")
            {
                mit.update(2, 1, 2001);
                mit.update(2, 2, 2002);
                mit.update(2, 3, 2003);
                mit.update(2, 4, 2004);
                mit.update(2, 5, 2005);
            }
            else
            {
                mit.update(3, 1, 3001);
                mit.update(3, 2, 3002);
                mit.update(3, 3, 3003);
                mit.update(3, 4, 3004);
                mit.update(3, 5, 3005);
            }

            // Broadcast this MIT.
            std::vector<uint8_t> mitSer = mit.serialise();
            AdvMsg msg(mitSer);
            transmit(ptp_comms::BROADCAST_ADDR, msg);

            ros::Duration(ADV_INTERVAL).sleep();
        }
    }

    /**
     * @brief Handles received bytes data.
     * 
     * @param src Source address of this bytes data.
     * @param data Data received.
     */
    void _handleRxData(std::string src, std::vector<uint8_t>& data)
    {
        MsgType t = _peekType(data);
        switch (t)
        {
        case MsgType::Data:
        {
            DataMsg dataM;
            dataM.deserialize(data);
            _handleData(dataM, src);
            break;
        }
        case MsgType::Request:
        {
            ReqMsg req;
            req.deserialize(data);
            _handleReq(req, src);
            break;
        }
        case MsgType::Acknowledgement:
        {
            AckMsg ack;
            ack.deserialize(data);
            _handleAck(ack, src);
            break;
        }
        default: // Advertisement
        {
            AdvMsg adv;
            adv.deserialize(data);
            _handleAdv(adv, src);
            break;
        }
        }
    }

    /**
     * @brief Peeks at the type of message the bytes vector contains.
     * 
     * @param data Bytes vector.
     * @return MsgType Type of message.
     */
    MsgType _peekType(std::vector<uint8_t>& data)
    {
        return (MsgType)data.data()[0];
    }

    /**
     * @brief Handles a DATA message.
     * 
     * @param msg DATA message to handle.
     * @param src Source address of DATA message.
     */
    void _handleData(DataMsg& msg, std::string src)
    {
        // std::cout << "GOT DATA" << std::endl;
        reqsMediator->notifyData(src, msg);
    }

    /**
     * @brief Handles a REQ message.
     * 
     * @param msg REQ message to handle.
     * @param src Source address of REQ message.
     */
    void _handleReq(ReqMsg& msg, std::string src)
    {
        respsMediator->queueResp(msg.reqSequence, msg.reqEntryId, src);
    }

    /**
     * @brief Handles an ACK message.
     * 
     * @param msg ACK message to handle.
     * @param src Source address of ACK message.
     */
    void _handleAck(AckMsg& msg, std::string src)
    {
        if (msg.forReq)
        {
            // std::cout << "GOT ACK FOR REQ" << std::endl;
            reqsMediator->notifyAck(src, msg);
        }
        else
        {
            // std::cout << "GOT ACK FOR DATA" << std::endl;
            respsMediator->notifyAck(src, msg);
        }
    }

    /**
     * @brief Handles an ADV message.
     * 
     * @param msg ADV message to handle.
     * @param src Source address of ACK message.
     */
    void _handleAdv(AdvMsg& msg, std::string src)
    {
        // std::cout << "GOT ADV" << std::endl;
        MIT mit;
        mit.deserialise(msg.data);

        // TODO: Compare incoming MIT with our own local one.
        std::cout << "GOT ADV from " << src << std::endl;
        // Some example request
        MIT mock;
        auto v = mock.compare(mit);
        for (auto val : v)
        {
            reqsMediator->queueReq(mock.convertToEntryId(val.first, val.second), src);
        }
    }

public:
    /**
     * @brief Construct a new DDP object.
     * 
     * @param nh ROS NodeHandle object used to create ROS transport structures.
     */
    DDP(ros::NodeHandle& nh)
    {
        ptpClient = std::unique_ptr<ptp_comms::PtpClient>(new ptp_comms::PtpClient(ptp_comms::DEFAULT_PORT));
        ptpClient->bind(
            std::bind(&DDP::_rxCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        reqsMediator = std::unique_ptr<ReqsMediator>(new ReqsMediator(this, this));
        respsMediator = std::unique_ptr<RespsMediator>(new RespsMediator(this, this));
        
        // Begin main thread operation.
        mainRunning.store(true);
        rxHandleTh = std::thread(&DDP::_runMain, this);

        // Begin advertisement operation
        advRunning.store(true);
        advTh = std::thread(&DDP::_runAdv, this);
    }

    /**
     * @brief Destroy the DDP object.
     * 
     */
    ~DDP()
    {
        ptpClient->unregister();
        mainRunning.store(false);
        advRunning.store(false);

        if (rxHandleTh.joinable())
        {
            rxHandleTh.join();
        }

        if (advTh.joinable())
        {
            advTh.join();
        }
    }

    /**
     * @brief Transmits message by calling the data transmission service.
     * 
     * @param dest Intended recipient of data.
     * @param msg Message to transmit.
     * @return true If call was successful.
     * @return false If call has failed.
     */
    bool transmit(std::string dest, BaseMsg& msg)
    {
        std::vector<uint8_t> data = msg.serialize();
        return ptpClient->sendTo(dest, data);
    }

    /**
     * @brief Pushes new data into the database node to store.
     * 
     * @param entryId EntryID of this data.
     * @param timestamp Timestamp of this data.
     * @param data Actual data to push.
     * @return true If the push was successul.
     * @return false If the push was unsuccessful.
     */
    bool pushData(uint16_t entryId, uint64_t timestamp, const std::vector<uint8_t>& data)
    {
        std::cout << "Pushing data for Entry ID " << entryId << " with timestamp " << timestamp << ": ";
        for (auto v : data)
        {
            printf("%u ", v);
        }
        std::cout << std::endl;
        return true;
    }

    /**
     * @brief Pulls a data from the database node.
     * 
     * @param entryId EntryID of data to pull.
     * @param timestamp Reference to store timestamp of pulled data.
     * @param data Reference to store pulled data.
     * @return true If the pull was successul.
     * @return false If the pull was unsuccessful.
     */
    bool pullData(uint16_t entryId, uint64_t& timestamp, std::vector<uint8_t>& data)
    {
        uint64_t mockTs = 1234;
        std::vector<uint8_t> mockData = {5, 6, 7, 8};
        timestamp = mockTs;
        data = mockData;

        std::cout << "Pulling data for Entry ID " << entryId << " with timestamp " << timestamp << ": ";
        for (auto v : data)
        {
            printf("%u ", v);
        }
        std::cout << std::endl;

        return true;
    }
};

#endif // H_DDP