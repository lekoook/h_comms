#include <memory>
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"
#include "ptp_comms/Neighbors.h"
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
     * @brief Maximum time between the last beacon broadcast received from a neighbor and the current time. Decides if a
     *  neighbor is alive and nearby recently.
     * @details This value should be sufficiently greater than PING_INTERVAL to allow some overhead from code execution.
     * 
     */
    const double MAX_NEIGHB_TIME = 1.5;

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
     * @brief ROS service server to allow querying of nearby neighbors.
     * 
     */
    ros::ServiceServer neighborsService;

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

    /**
     * @brief Callback for neighbors query service calls.
     * 
     * @param req Request message.
     * @param resp Response message.
     * @return true If there is at least one neighbor.
     * @return false If there is no neighbor.
     */
    bool _neighb(ptp_comms::Neighbors::Request& req, ptp_comms::Neighbors::Response& resp)
    {
        Neighbor_M nb = cc->neighbors();
        for (auto it : nb)
        {
            if ((ros::Time::now().toSec() - it.second.first < MAX_NEIGHB_TIME) && !std::isinf(it.second.second))
            {
                resp.names.push_back(it.first);
                resp.timestamps.push_back(it.second.first);
                resp.rssi.push_back(it.second.second);
            }
        }
        return resp.names.size() > 0;
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
        
        neighborsService = nh.advertiseService<ptp_comms::Neighbors::Request, ptp_comms::Neighbors::Response>(
            "neighbors",
            std::bind(&PTP::_neighb, this, std::placeholders::_1, std::placeholders::_2)
            );
    }
};

}
