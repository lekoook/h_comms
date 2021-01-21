#ifndef H_PTP_CLIENT
#define H_PTP_CLIENT

#include "ros/ros.h"
#include <cstdint>
#include "ptp_comms/RegisterPort.h"
#include "ptp_comms/RxData.h"
#include "ptp_comms/TxData.h"
#include "ptp_comms/Neighbors.h"
#include "CommonTypes.hpp"

namespace ptp_comms
{

/**
 * @brief This is an interface to provide simplified access to transmit and receive data via the underlying transport
 * protocol (ptp_comms).
 * 
 * @details For each application (purpose) use, you should instantiate a PtpClient with a unique port number for that 
 * application. Data sent through this PtpClient with this port number will be received on the receiver side with the
 * a PtpClient using the same port number. This allows multiple applications to use the same underlying transport 
 * protocol.
 * 
 * When a PtpClient is instantiated, the port number is used to register with the transport protocol's registry and data
 * received from that port number will be published on a dedicated ROS topic identified by `rx_data/[port]`. Therefore,
 * it is vital that PtpClient instance creators should call unregister() to unregister the port number from the 
 * transport protocol's registry before the termination of program. This is to free up that port number for reuse. 
 * Otherwise, that port number is permanently blocked from use.
 */
class PtpClient
{
private:
    /**
     * @brief Maximum number of messages in the publisher queue.
     * 
     */
    const uint32_t MAX_QUEUE_SIZE = 100;

    /**
     * @brief Time between each attempt to register at constructor.
     * 
     */
    const double RETRY_INTERVAL = 3.0f;

    /**
     * @brief Prefix string for all reception topics.
     * 
     */
    const std::string RX_TOPIC_PREFIX = "rx_data/";

    /**
     * @brief Prefix string for transmission topic.
     * 
     */
    const std::string TX_SVC_PREFIX = "tx_data";

    /**
     * @brief Prefix string for neighbors query service.
     * 
     */
    const std::string NEIGHB_SVC_PREFIX = "neighbors";

    /**
     * @brief Address that this client is using.
     * 
     */
    std::string localAddr;

    /**
     * @brief Port number this client will use.
     * 
     */
    uint16_t localPort;

    /**
     * @brief Callback that will be called when data is received.
     * 
     */
    std::function<void(std::string src, uint16_t, std::vector<uint8_t>)> cb;

    /**
     * @brief ROS subscriber to the topic for receiving data.
     * 
     */
    ros::Subscriber subber;

    /**
     * @brief ROS service for transmitting data.
     * 
     */
    ros::ServiceClient txSvc;

    /**
     * @brief ROS service for querying neighbors.
     * 
     */
    ros::ServiceClient neighbSvc;

    /**
     * @brief Registers the port number with port registry.
     * 
     * @return true If registration successful.
     * @return false If registration failed.
     */
    bool _reg()
    {
        ros::NodeHandle nh;
        ros::ServiceClient regSvc = nh.serviceClient<ptp_comms::RegisterPort>("register");
        ptp_comms::RegisterPort msg;
        msg.request.port = localPort;

        if (regSvc.call(msg))
        {
            subber = nh.subscribe(RX_TOPIC_PREFIX + std::to_string(localPort), MAX_QUEUE_SIZE, &PtpClient::_rxCb, this);
            txSvc = nh.serviceClient<ptp_comms::TxData>(TX_SVC_PREFIX);
            neighbSvc = nh.serviceClient<ptp_comms::Neighbors>(NEIGHB_SVC_PREFIX);
            localAddr = msg.response.address;
            ROS_INFO("PtpClient registration for port %u success.", localPort);
            return true;
        }
        else
        {
            ROS_ERROR("PtpClient registration for port %u failed. Reason: %s.", localPort, msg.response.reason.c_str());
            return false;
        }
    }

    /**
     * @brief Unregisters the port number with port registry.
     * 
     * @return true If unregistration successful.
     * @return false If unregistration failed.
     */
    bool _unreg()
    {
        ros::NodeHandle nh;
        ros::ServiceClient regSvc = nh.serviceClient<ptp_comms::RegisterPort>("unregister");
        ptp_comms::RegisterPort msg;
        msg.request.port = localPort;

        if (regSvc.call(msg))
        {
            subber.shutdown();
            txSvc.shutdown();
            ROS_INFO("PtpClient unregistration for port %u success.", localPort);
            return true;
        }
        else
        {
            ROS_ERROR("PtpClient unregistration for port %u failed. Reason: %s.", localPort, msg.response.reason.c_str());
            return false;
        }
    }

    /**
     * @brief Subscriber callback for receiving data.
     * 
     * @param msg Message containing the data.
     */
    void _rxCb(const ptp_comms::RxDataConstPtr& msg)
    {
        if (cb)
        {
            cb(msg->src, msg->port, msg->data);
        }
    }

public:
    /**
     * @brief Construct a new Ptp Client object.
     * 
     * @param port Port number to use.
     * @param autoRetry If set to true, this call will be blocking until registration is successful.
     */
    PtpClient(uint16_t port, bool autoRetry=false) : localPort(port)
    {
        while (!_reg())
        {
            if (!autoRetry)
            {
                break;
            }
            ros::Duration(RETRY_INTERVAL).sleep();
        }
    }

    /**
     * @brief Destroy the Ptp Client object.
     * 
     */
    ~PtpClient()
    {
        _unreg();
    }

    /**
     * @brief Binds the calllback to call when data is received.
     * 
     * @param callback Callback to call.
     */
    void bind(std::function<void(std::string src, uint16_t, std::vector<uint8_t>)> callback)
    {
        cb = callback;
    }

    /**
     * @brief Gets the address associated with this client.
     * 
     * @return std::string Address associated with this client.
     */
    std::string address()
    {
        return localAddr;
    }

    /**
     * @brief Gets the port number associated with this client.
     * 
     * @return uint16_t Port number associated with this client.
     */
    uint16_t port()
    {
        return localPort;
    }

    /**
     * @brief Attempts to send data to the destination.
     * 
     * @param dest Intended destination address of data.
     * @param data Data to send.
     * @return true If successful.
     * @return false If unsuccessful.
     */
    bool sendTo(std::string dest, std::vector<uint8_t> data)
    {
        if (txSvc.exists())
        {
            ptp_comms::TxData msg;
            msg.request.dest = dest;
            msg.request.port = localPort;
            msg.request.data = data;
            return txSvc.call(msg);
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Unregisters the client to free up the port number.
     * 
     * @return true If successful.
     * @return false If unsuccessful.
     */
    bool unregister()
    {
        return _unreg();
    }

    /**
     * @brief Queries all nearby and recent neighbors of this client.
     * 
     * @return Neighbor_M A map of all nearby neighbors and their information (timestamp of beacon broadcast and RSSI).
     */
    Neighbor_M neighbors()
    {
        Neighbors msg;
        if (neighbSvc.isValid() && neighbSvc.call(msg))
        {
            Neighbor_M ret;
            for (int i = 0; i < msg.response.names.size(); i++)
            {
                ret[msg.response.names[i]] = std::make_pair(msg.response.timestamps[i], msg.response.rssi[i]);
            }
            return ret;
        }
        return Neighbor_M();
    }

    /**
     * @brief Queries if an address is a neighbor of this client.
     * @details A neighbor is defined to be a node that this client has received a ping from recently.
     * 
     * @param address Address to check.
     * @return true If the address is a neighbor.
     * @return false If the address is not a neighbor.
     */
    bool isNeighbor(std::string address)
    {
        auto n = neighbors();
        return n.find(address) != n.end();
    }
};

}

#endif // H_PTP_CLIENT