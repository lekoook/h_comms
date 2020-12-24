#ifndef H_CTRANSCEIVER
#define H_CTRANSCEIVER

#include <memory>
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
     * @brief subt::CommsClient used to transmit and receive data.
     * 
     */
    std::unique_ptr<subt::CommsClient> cc;

    /**
     * @brief The local address of this transceiver.
     * 
     */
    const std::string localAddr;
    
public:
    /**
     * @brief Construct a new CTransceiver object.
     * 
     * @param address The local address of this transceiver.
     */
    CTransceiver(const std::string& address) 
        : localAddr(address)
        , cc(std::unique_ptr<subt::CommsClient>(new subt::CommsClient(address))) {}

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
    ptp_comms::Neighbor_M neighbors() const
    {
        return cc->Neighbors();
    }

    /**
     * @brief Sends a neighbour discovery broadcast.
     * 
     * @return true If send was successful.
     * @return false If send was unsuccessful.
     */
    bool sendBeacon()
    {
        return cc->SendBeacon();
    }

    /**
     * @brief Sends neighbour discovery broadcast at a regular interval to stimulate neighbours discovery.
     * 
     * @param period Time interval to send the broadcast.
     */
    void startBeaconInterval(ros::Duration period)
    {
        cc->StartBeaconInterval(period);
    }
};

}

#endif // H_CTRANSCEIVER