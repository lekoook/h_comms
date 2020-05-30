#include "comms_time.h"

namespace h_comms
{
    uint64_t time_ms()
    {
        #ifdef COMMS_USE_ROS_TIME
        return ros::Time::now().toNSec() / 1000000lu;
        #else
        return 0;
        #endif 
    }
}