#ifndef H_PTP_COMMS_COMMON_TYPES
#define H_PTP_COMMS_COMMON_TYPES

#include <string>
#include <map>

namespace ptp_comms
{
    const uint16_t DEFAULT_PORT = 4100u;
    const std::string BROADCAST_ADDR = "broadcast";
    typedef std::map<std::string, std::pair<double, double>> Neighbor_M;
}

#endif // H_PTP_COMMS_COMMON_TYPES