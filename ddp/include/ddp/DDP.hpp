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
#include "MsgHandler.hpp"
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
     * @brief Processes all received messages accordingly from their type and fields information.
     * 
     */
    MsgHandler msgHandler;

    /**
     * @brief Mediator for one or more Requestors.
     * 
     */
    ReqsMediator reqsMediator;

    /**
     * @brief Mediator for one or more Responders.
     * 
     */
    RespsMediator respsMediator;

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
            bool available = false;
            RxQueueData rxData;
            {
                std::lock_guard<std::mutex> qLock(mRxQ);
                if (!rxQ.empty())
                {
                    available = true;
                    rxData = rxQ.front();
                    rxQ.pop();
                }
            }

            if (available)
            {
                msgHandler.notifyRx(rxData.src, rxData.data);
            }
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

            // Broadcast this MIT.
            std::vector<uint8_t> mitSer = mit.serialise();
            AdvMsg msg(mitSer);
            transmit(ptp_comms::BROADCAST_ADDR, msg);

            ros::Duration(ADV_INTERVAL).sleep();
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
        reqsMediator.start(this);
        respsMediator.start(this);
        msgHandler = MsgHandler(&reqsMediator, &respsMediator);
        
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
        return true;
    }
};

#endif // H_DDP