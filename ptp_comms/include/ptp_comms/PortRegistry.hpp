#include "ros/ros.h"
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include "ptp_comms/RxData.h"
#include "ptp_comms/RegisterPort.h"
#include "subt_communication_broker/subt_communication_client.h"

/**
 * @brief Represents the registry that will track all registerations of port numbers.
 * 
 */
class PortRegistry
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
     * @brief Contains a record of all registered port numbers.
     * 
     */
    std::map<uint16_t, ros::Publisher> registry;

    /**
     * @brief Mutex to protect registry.
     * 
     */
    std::mutex mRegistry;

    /**
     * @brief Pointer to a ROS NodeHandle. Used to advertise topics.
     * 
     */
    ros::NodeHandle* nh;

    /**
     * @brief subt::CommsClient used to bind callbacks to a port.
     * 
     */
    subt::CommsClient* cc;

    /**
     * @brief ROS service server to handle registering of port numbers.
     * 
     */
    ros::ServiceServer registerService;

    /**
     * @brief ROS service server to handle unregistering of port numbers.
     * 
     */
    ros::ServiceServer unregisterService;

    /**
     * @brief Local address that should be used to bind callbacks to a port.
     * 
     */
    std::string localAddr;

    /**
     * @brief Callback function that should be used to bind callbacks to a port.
     * 
     */
    std::function<void(const std::string&, const std::string&, const uint32_t, const std::string&)> rxCallback;

    /**
     * @brief Callback for the port registering service calls.
     * 
     * @param req Request message.
     * @param res Response message.
     * @return true If port was successfully registered.
     * @return false If port was not registered.
     */
    bool _regPort(ptp_comms::RegisterPort::Request &req, ptp_comms::RegisterPort::Response &res)
    {
        res.successful = _registerPort(req.port);
        if (!res.successful)
        {
            res.reason = "Port is already registered!";
        }
        return res.successful;
    }

    /**
     * @brief Callback for the port unregistering service calls.
     * 
     * @param req Request message.
     * @param res Response message.
     * @return true If port was successfully unregistered.
     * @return false If port was not unregistered.
     */
    bool _unregPort(ptp_comms::RegisterPort::Request &req, ptp_comms::RegisterPort::Response &res)
    {
        res.successful = _unregisterPort(req.port);
        if (!res.successful)
        {
            res.reason = "Port is not registered in the first place!";
        }
        return res.successful;
    }

    /**
     * @brief Registers a port number and creates a RX publisher associated with that port.
     * 
     * @param port Port number to register.
     * @return true If the port is not already registered and register was successful.
     * @return false If the port is already registered.
     */
    bool _registerPort(uint16_t port)
    {
        bool missing = registry.find(port) == registry.end();
        if (missing)
        {
            std::lock_guard<std::mutex> lock(mRegistry);
            registry[port] = nh->advertise<ptp_comms::RxData>(TOPIC_PREFIX + std::to_string(port), MAX_QUEUE_SIZE);
            if (cc->Bind(rxCallback, localAddr, port))
            {
                ROS_INFO("CommsClient binded to address: %s , port: %u", localAddr.c_str(), port);
            }
            else
            {
                ROS_ERROR("Cannot bind CommsClient. Could it have already been binded before?");
            }
            
            return true;
        }
        return missing;
    }

    /**
     * @brief Unregisters a port number and removes the RX publisher associated with that port.
     * 
     * @param port Port number to unregister.
     * @return true If the port was already registered and unregister was successful.
     * @return false If the port was not registered in the first place.
     */
    bool _unregisterPort(uint16_t port)
    {
        std::lock_guard<std::mutex> lock(mRegistry);
        return registry.erase(port) > 0;
    }

public:
    /**
     * @brief Construct a new Port Registry object.
     * 
     * @param nodeHandle NodeHandle used to construct publishers.
     * @param commsClient subt::CommsClient used to bind callbacks.
     * @param localAddr Local address of this node.
     * @param rxCallback Callback to bind for subt::CommsClient.
     */
    PortRegistry(
        ros::NodeHandle* nodeHandle, 
        subt::CommsClient* commsClient, 
        std::string localAddr, 
        std::function<void(const std::string&, const std::string&, const uint32_t, const std::string&)> rxCallback) 
        : nh(nodeHandle), cc(commsClient), localAddr(localAddr), rxCallback(rxCallback)
    {
        registerService = nh->advertiseService<ptp_comms::RegisterPort::Request, ptp_comms::RegisterPort::Response>(
            "register",
            std::bind(&PortRegistry::_regPort, this, std::placeholders::_1, std::placeholders::_2)
            );
        unregisterService = nh->advertiseService<ptp_comms::RegisterPort::Request, ptp_comms::RegisterPort::Response>(
            "unregister",
            std::bind(&PortRegistry::_unregPort, this, std::placeholders::_1, std::placeholders::_2)
            );
    }

    /**
     * @brief Check if a port was registered.
     * 
     * @param port Port to check.
     * @return true If the port was registered.
     * @return false If the port was not registered.
     */
    bool portIsRegistered(uint16_t port)
    {
        std::lock_guard<std::mutex> lock(mRegistry);
        return registry.find(port) != registry.end();
    }

    /**
     * @brief Gets a pointer to the RX publisher for the port.
     * 
     * @param port Port of the RX publisher.
     * @param publisher Pointer to store the address of the publisher.
     * @return true If the port was not registered in the first place.
     * @return false If the port was registered.
     */
    bool getPublisher(uint16_t port, std::map<uint16_t, ros::Publisher>::iterator& publisher)
    {
        std::lock_guard<std::mutex> lock(mRegistry);
        auto it = registry.find(port);
        if (it == registry.end())
        {
            return false;
        }
        else
        {
            publisher = it;
            return true;
        }
    }
};