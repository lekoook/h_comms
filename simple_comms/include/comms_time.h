#ifndef COMMS_TIME_H
#define COMMS_TIME_H

#define COMMS_USE_ROS_TIME // Use ROS time

#ifdef COMMS_USE_ROS_TIME
#include <ros/time.h>
#else
#include <inttypes.h>
#endif

namespace h_comms
{
    uint64_t time_ms();
}

#endif // COMMS_TIME_H