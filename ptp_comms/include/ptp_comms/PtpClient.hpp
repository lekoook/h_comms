#include "ros/ros.h"
#include <cstdint>
#include "ptp_comms/RegisterPort.h"
#include "ptp_comms/RxData.h"

class PtpClient
{
private:
    /**
     * @brief Maximum number of messages in the publisher queue.
     * 
     */
    const uint32_t MAX_QUEUE_SIZE = 100;

    /**
     * @brief Prefix string for all topics.
     * 
     */
    const std::string TOPIC_PREFIX = "rx_data/";

    /**
     * @brief Port number this client will use.
     * 
     */
    uint16_t port;

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
        msg.request.port = port;

        if (regSvc.call(msg))
        {
            subber = nh.subscribe(TOPIC_PREFIX + std::to_string(port), MAX_QUEUE_SIZE, &PtpClient::_rxCb, this);
            ROS_INFO("PtpClient registration for port %u success.", port);
            return true;
        }
        else
        {
            ROS_ERROR("PtpClient registration for port %u failed. Reason: %s.", port, msg.response.reason.c_str());
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
        msg.request.port = port;

        if (regSvc.call(msg))
        {
            subber.shutdown();
            ROS_INFO("PtpClient unregistration for port %u success", port);
            return true;
        }
        else
        {
            ROS_ERROR("PtpClient unregistration for port %u failed", port);
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
        cb(msg->src, msg->port, msg->data);
    }

public:
    /**
     * @brief Construct a new Ptp Client object.
     * 
     * @param port Port number to use.
     */
    PtpClient(uint16_t port) : port(port)
    {
        _reg();
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
    void Bind(std::function<void(std::string src, uint16_t, std::vector<uint8_t>)> callback)
    {
        cb = callback;
    }
};