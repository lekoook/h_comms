#include <memory>
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"
#include "TxerTh.hpp"
#include "RxerTh.hpp"
#include "PortRegistry.hpp"
#include "ATransceiver.hpp"
#include "CTransceiver.hpp"

namespace ptp_comms
{

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
     * @brief ROS service server to respond to transmission requests.
     * 
     */
    ros::ServiceServer txService;

    /**
     * @brief Interface to transmit and receive data.
     * 
     */
    std::unique_ptr<ATransceiver> cc;

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
            if (!portRegistry->portIsRegistered(_dstPort))
            {
                ROS_INFO("Incoming data for unregistered port: %u", _dstPort);
            }
            else
            {
                while(!rxTh->recvOne(RxQueueData(_data, _srcAddress, _dstAddress, _dstPort)));
            }
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
        if (!portRegistry->portIsRegistered(req.port))
        {
            res.successful = false;
            res.reason = "Port is not registered!";
            return false;
        }
        
        if (req.dest != nodeAddr)
        {
            res.successful = txTh->sendOne(req.data, req.dest, req.port);
        }
        else
        {
            res.successful = false;
            res.reason = "Cannot send to self!";
        }
        
        return res.successful;
    }

    /**
     * @brief Callback that should be called to publish received data.
     * 
     * @param src Source address of the data.
     * @param port Port number of the data.
     * @param data Actual data received.
     */
    void _pubRx(std::string src, uint16_t port, std::vector<uint8_t> data)
    {
        std::map<uint16_t, ros::Publisher>::iterator it;
        if (portRegistry->getPublisher(port, it))
        {
            ptp_comms::RxData msg;
            msg.data = data;
            msg.src = src;
            msg.port = port;
            it->second.publish(msg);
        }
        else
        {
            ROS_ERROR("Cannot find publisher for port: %u", port);
        }
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
        cc = std::unique_ptr<ATransceiver>(new CTransceiver(nodeAddr));
        txTh = std::unique_ptr<TxerTh>(new TxerTh(cc.get()));
        rxTh = std::unique_ptr<RxerTh>(new RxerTh(
            cc.get(), 
            txTh.get(), 
            std::bind(&PTP::_pubRx, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));

        cc->startBeaconInterval(ros::Duration(PING_INTERVAL));
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

}
