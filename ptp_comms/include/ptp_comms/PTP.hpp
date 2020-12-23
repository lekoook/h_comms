#include <memory>
#include "subt_communication_broker/subt_communication_client.h"
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"
#include "TxerTh.hpp"
#include "RxerTh.hpp"
#include "PortRegistry.hpp"

/**
 * @brief Class that encapsulates the point-to-point communication exchange sequences.
 * 
 */
class PTP
{
private:
    // Constants
    /**
     * @brief Interval (seconds) between each subt::CommsClient beacon broadcast.
     * 
     */
    const double PING_INTERVAL = 1.0;

    /**
     * @brief Local address of this node.
     * 
     */
    std::string nodeAddr;

    /**
     * @brief ROS publisher to publish received data.
     * 
     */
    ros::Publisher rxPubber;

    /**
     * @brief ROS service server to respond to transmission requests.
     * 
     */
    ros::ServiceServer txService;

    /**
     * @brief subt::CommsClient to transmit and receive data.
     * 
     */
    std::unique_ptr<subt::CommsClient> cc;

    /**
     * @brief Transmission thread that will handle all messages to be transmitted.
     * 
     */
    std::unique_ptr<TxerTh> txTh;

    /**
     * @brief Reception thread that will handle all received messages.
     * 
     */
    std::unique_ptr<RxerTh> rxTh;

    /**
     * @brief Registry to track all registerations of port numbers.
     * 
     */
    std::unique_ptr<PortRegistry> portRegistry;

    /**
     * @brief Callback for receive data from subt::CommsClient.
     * 
     * @param _srcAddress Source address of the data.
     * @param _dstAddress Destination address of the data.
     * @param _dstPort Destination port of the data.
     * @param _data Actual data received.
     */
    void _rxCb(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
    {
        if (_srcAddress != nodeAddr)
        {
            while(!rxTh->recvOne(RxQueueData(_data, _srcAddress, _dstAddress)));
        }
    }

    /**
     * @brief Callback for transmission service request calls.
     * 
     * @param req Request message.
     * @param res Response message.
     * @return true If data was successfully queued.
     * @return false If data was not queued.
     */
    bool _txData(ptp_comms::TxData::Request &req, ptp_comms::TxData::Response &res)
    {
        if (req.dest != nodeAddr)
        {
            res.successful = txTh->sendOne(req.data, req.dest);
        }
        else
        {
            res.successful = false;
        }
        
        return res.successful;
    }

    /**
     * @brief Callback that should be called to publish received data.
     * 
     * @param src Source address of the data.
     * @param data Actual data received.
     */
    void _pubRx(std::string src, std::vector<uint8_t> data)
    {
        ptp_comms::RxData msg;
        msg.data = data;
        msg.src = src;
        rxPubber.publish(msg);
    }

public:
    /**
     * @brief Construct a new PTP object.
     * 
     * @param nh ROS NodeHandle.
     * @param nodeAddr Address for this node.
     */
    PTP(ros::NodeHandle& nh, std::string nodeAddr) : nodeAddr(nodeAddr)
    {
        // Set up communication drivers.
        cc = std::unique_ptr<subt::CommsClient>(new subt::CommsClient(nodeAddr));
        txTh = std::unique_ptr<TxerTh>(new TxerTh(cc.get()));
        rxTh = std::unique_ptr<RxerTh>(new RxerTh(
            cc.get(), 
            txTh.get(), 
            std::bind(&PTP::_pubRx, this, std::placeholders::_1, std::placeholders::_2)));

        cc->StartBeaconInterval(ros::Duration(PING_INTERVAL));
        txTh->start();
        rxTh->start();

        // Set up ROS structures.
        portRegistry = std::unique_ptr<PortRegistry>(new PortRegistry(&nh, cc.get(), nodeAddr, 
            std::bind(
                &PTP::_rxCb, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2, 
                std::placeholders::_3, 
                std::placeholders::_4
                )));
        txService = nh.advertiseService<ptp_comms::TxData::Request, ptp_comms::TxData::Response>(
            "tx_data", 
            std::bind(&PTP::_txData, this, std::placeholders::_1, std::placeholders::_2)
            );
    }
};