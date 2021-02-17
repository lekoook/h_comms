#ifndef H_CTRANSCEIVER
#define H_CTRANSCEIVER

#include <memory>
#include <thread>
#include <functional>
#include "ATransceiver.hpp"
#include "subt_communication_broker/subt_communication_client.h"

namespace ptp_comms
{

/**
 * @brief Concrete implementation of the ATransceiver interface.
 * 
 */
class CTransceiver : public ATransceiver 
{
private:
    /**
     * @brief Message used to identify a broadcasted advertisement.
     * 
     */
    std::string const ADV_MSG = "discovery";

    /**
     * @brief The address used to broadcast advertisement.
     * 
     */
    std::string const ADV_ADDR = subt::communication_broker::kBroadcast;

    /**
     * @brief The port used to broadcast advertisement.
     * 
     */
    uint32_t const ADV_PORT = 100;

    /**
     * @brief subt::CommsClient used to transmit and receive data.
     * 
     */
    std::unique_ptr<subt::CommsClient> cc;

    /**
     * @brief The local address of this transceiver.
     * 
     */
    const std::string localAddr;
    
    /**
     * @brief Thread used to broadcast advertisements as a beacon.
     * 
     */
    std::thread beaconTh;

    /**
     * @brief Flag that determines if the thread to broadtcast advertisement for discovery should run.
     * 
     */
    std::atomic_bool runBeacon;

    /**
     * @brief Record of all advertisements received from neighboring nodes.
     * 
     */
    ptp_comms::Neighbor_M neighborsRecord;

    /**
     * @brief Mutex to protect the neighbors record.
     * 
     */
    std::mutex mNeighbors;

    /**
     * @brief Function to run for the beacon thread.
     * 
     * @param period The time interval between each advertisement broadcast.
     */
    void _advertise(ros::Duration period)
    {
        if (runBeacon)
        {
            _advertiseOnce();
            period.sleep();
        }
    }

    /**
     * @brief Broadcast a single advertisement message once. 
     * 
     */
    bool _advertiseOnce()
    {
        return cc->SendTo(ADV_MSG, ADV_ADDR, ADV_PORT);
    }

    /**
     * @brief Callback for receiving advertisement messages.
     * 
     * @param srcAddress Source address of advertisement.
     * @param destAddress Destination address of advertisement, should default to \p ADV_ADDR.
     * @param dstPort Port number of advertisement, should default to \p ADV_PORT.
     * @param _data Contains the advertisement message, should default to \p ADV_MSG.
     */
    void _advCallback(const std::string& srcAddress, 
        const std::string& destAddress, 
        const uint32_t dstPort, 
        const std::string &_data)
    {
        if (_data == ADV_MSG)
        {
            double ts = ros::Time::now().toSec();
            std::lock_guard<std::mutex> lock(mNeighbors);
            neighborsRecord[srcAddress] = std::make_pair(ts, 0.0); // We can't get RSSI information at the moment, use zero.
        }
    }
    
public:
    /**
     * @brief Construct a new CTransceiver object.
     * 
     * @param address The local address of this transceiver.
     */
    CTransceiver(const std::string& address) 
        : localAddr(address)
        , cc(std::unique_ptr<subt::CommsClient>(new subt::CommsClient(address)))
    {
        cc->Bind(std::bind(
                &CTransceiver::_advCallback, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2, 
                std::placeholders::_3, 
                std::placeholders::_4), 
            ADV_ADDR, 
            ADV_PORT);
    }

    /**
     * @brief Returns the local address.
     * 
     * @return std::string Local address.
     */
    std::string host() const
    {
        return localAddr;
    }
    
    /**
     * @brief Binds a callback to incoming data intended for the address and port.
     * 
     * @param callback Callback to call when incoming data arrives.
     * @param address Address for which the data is intended for.
     * @param port Port for which the data is intended for.
     * @return true If bind was successful.
     * @return false If bind was unsuccessful.
     */
    bool bind(std::function<void(const std::string& srcAddress,
                                         const std::string& destAddress,
                                         const uint32_t dstPort,
                                         const std::string &_data)> callback,
                      const std::string &address,
                      const uint16_t port = ptp_comms::DEFAULT_PORT)
    {
        return cc->Bind(callback, address, port);
    }

    /**
     * @brief Sends data to the specific address and port.
     * 
     * @param data Data to send.
     * @param dstAddress Intended destination address of the data.
     * @param port Intended port of the data.
     * @return true If send was successful.
     * @return false If send was unsuccessful.
     */
    bool sendTo(const std::string& data,
                        const std::string& dstAddress,
                        const uint16_t port = ptp_comms::DEFAULT_PORT)
    {
        return cc->SendTo(data, dstAddress, port);
    }

    /**
     * @brief Returns the neighbours nearby.
     * 
     * @return Neighbor_M Map of neighbours.
     */
    ptp_comms::Neighbor_M neighbors()
    {
        std::lock_guard<std::mutex> lock(mNeighbors);
        return neighborsRecord;
    }

    /**
     * @brief Sends a neighbour discovery broadcast.
     * 
     * @return true If send was successful.
     * @return false If send was unsuccessful.
     */
    bool sendBeacon()
    {
        return _advertiseOnce();
    }

    /**
     * @brief Sends neighbour discovery broadcast at a regular interval to stimulate neighbours discovery.
     * 
     * @param period Time interval to send the broadcast.
     */
    void startBeaconInterval(ros::Duration period)
    {
        if (runBeacon && beaconTh.joinable())
        {
            runBeacon = false;
            beaconTh.join();
        }
        runBeacon = true;
        beaconTh = std::thread(&CTransceiver::_advertise, this, period);
    }
};

}

#endif // H_CTRANSCEIVER