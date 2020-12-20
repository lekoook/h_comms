#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <string>
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"
#include "messages/AdvMsg.hpp"
#include "ATransmitter.hpp"
#include "MIT.hpp"
#include "MsgHandler.hpp"

class DDP : private ATransmitter
{
private:
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
    const std::string BROADCAST_ADDR = "broadcast"; // In accordance with subt API
    const double ADV_INTERVAL = 5.0;

    ros::ServiceClient msgSrvC;
    ros::Subscriber msgSubber;
    std::thread mainTh;
    std::atomic<bool> mainRunning;
    std::queue<RxQueueData> rxQ;
    std::mutex mRxQ;

    std::thread advTh;
    std::atomic<bool> advRunning;

    MsgHandler msgHandler;

    /**
     * @brief Subscriber callback to receive data. Puts the received data in a reception queue.
     * 
     * @param msg Contains the recieved data.
     */
    void _subCb(const ptp_comms::RxData::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mRxQ);
        rxQ.push(RxQueueData(msg->data, msg->src));
    }

    void _runMain()
    {
        // Main thread execution.
        while(mainRunning.load())
        {
            bool empty = false;
            {
                std::lock_guard<std::mutex> qLock(mRxQ);
                empty = rxQ.empty();
            }
            if (!empty)
            {
                RxQueueData rxData;
                {
                    std::lock_guard<std::mutex> qLock(mRxQ);
                    rxData = rxQ.front();
                    rxQ.pop();
                }

                msgHandler.notifyRx(rxData.src, rxData.data);
            }
        }
    }

    void _runAdv()
    {
        // Advertisement operation
        while(advRunning.load())
        {
            // TODO: Request MIT from ROS Service
            MIT mit;
            mit.update(1, 2, 3);

            // Broadcast this MIT.
            std::vector<uint8_t> mitSer = mit.serialise();
            std::cout << std::endl;
            AdvMsg msg(mitSer);
            transmit(BROADCAST_ADDR, msg);

            ros::Duration(ADV_INTERVAL).sleep();
        }
    }

public:
    DDP(ros::NodeHandle& nh)
    {
        // ROS subscriber and service client
        msgSrvC = nh.serviceClient<ptp_comms::TxData>("tx_data");
        msgSubber = nh.subscribe<ptp_comms::RxData>("rx_data", 100, &DDP::_subCb, this);

        msgHandler = MsgHandler(this);
        
        // Begin main thread operation.
        mainRunning.store(true);
        mainTh = std::thread(&DDP::_runMain, this);

        // Begin advertisement operation
        advRunning.store(true);
        advTh = std::thread(&DDP::_runAdv, this);
    }

    ~DDP()
    {
        mainRunning.store(false);
        advRunning.store(false);

        if (mainTh.joinable())
        {
            mainTh.join();
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
        ptp_comms::TxData tmsg;
        tmsg.request.data = data;
        tmsg.request.dest = dest;
        return msgSrvC.call(tmsg);
    }
};