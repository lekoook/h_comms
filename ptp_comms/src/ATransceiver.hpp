#ifndef H_ATRANSCEIVER
#define H_ATRANSCEIVER

#include "ros/ros.h"
#include <string>
#include <functional>

namespace ptp_comms
{
    const uint16_t DEFAULT_PORT = 4100u;
    const std::string BROADCAST_ADDR = "broadcast";
    typedef std::map<std::string, std::pair<double, double>> Neighbor_M;
}

/**
 * @brief Interface that provides the capability to transmit and receive data.
 * 
 */
class ATransceiver
{
public:
    /**
     * @brief Returns the local address.
     * 
     * @return std::string Local address.
     */
    virtual std::string host() const = 0;

    /**
     * @brief Binds a callback to incoming data intended for the address and port.
     * 
     * @param callback Callback to call when incoming data arrives.
     * @param address Address for which the data is intended for.
     * @param port Port for which the data is intended for.
     * @return true If bind was successful.
     * @return false If bind was unsuccessful.
     */
    virtual bool bind(std::function<void(const std::string& srcAddress,
                                         const std::string& destAddress,
                                         const uint32_t dstPort,
                                         const std::string &_data)> callback,
                      const std::string &address,
                      const uint16_t port = ptp_comms::DEFAULT_PORT) = 0;

    /**
     * @brief Sends data to the specific address and port.
     * 
     * @param data Data to send.
     * @param dstAddress Intended destination address of the data.
     * @param port Intended port of the data.
     * @return true If send was successful.
     * @return false If send was unsuccessful.
     */
    virtual bool sendTo(const std::string& data,
                        const std::string& dstAddress,
                        const uint16_t port = ptp_comms::DEFAULT_PORT) = 0;

    /**
     * @brief Returns the neighbours nearby.
     * 
     * @return Neighbor_M Map of neighbours.
     */
    virtual ptp_comms::Neighbor_M neighbors() const = 0;

    /**
     * @brief Sends a neighbour discovery broadcast.
     * 
     * @return true If send was successful.
     * @return false If send was unsuccessful.
     */
    virtual bool sendBeacon() = 0;

    /**
     * @brief Sends neighbour discovery broadcast at a regular interval to stimulate neighbours discovery.
     * 
     * @param period Time interval to send the broadcast.
     */
    virtual void startBeaconInterval(ros::Duration period) = 0;
};

#endif // H_ATRANSCEIVER