#ifndef H_PTP_COMMS_COMMON_TYPES
#define H_PTP_COMMS_COMMON_TYPES

#include <string>
#include <map>

namespace ptp_comms
{
    /**
     * @brief Default port of point to point communications protocol.
     * 
     */
    const uint16_t DEFAULT_PORT = 4100u;

    /**
     * @brief Standard address used for a broadcast message.
     * 
     */
    const std::string BROADCAST_ADDR = "broadcast";

    /**
     * @brief Map of all nearby neighbors. Key is the string name of neighbors. Value is a pair of (in order) timestamp 
     * of last received beacon broadcast and RSSI of that beacon broadcast.
     * 
     */
    typedef std::map<std::string, std::pair<double, double>> Neighbor_M;
}

#endif // H_PTP_COMMS_COMMON_TYPES