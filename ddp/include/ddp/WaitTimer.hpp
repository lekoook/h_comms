#ifndef H_WAIT_TIMER
#define H_WAIT_TIMER

#include "ros/ros.h"
#include <condition_variable>

class WaitTimer
{
private:
    double waitTime;
    ros::Timer timer;
    std::mutex mTimer;
    std::condition_variable cvWait;
    std::mutex mWait;
    std::atomic<bool> interrupted;
    std::atomic<bool> timeout;
    
    void timerCallback(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mWait);
        timeout = true;
        cvWait.notify_one();
    }
    
public:
    WaitTimer() {}

    WaitTimer(double waitTime) : waitTime(waitTime)
    {
        interrupted = false;
        timeout = false;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
    }

    WaitTimer& operator=(const WaitTimer& other)
    {
        interrupted = false;
        timeout = false;
        waitTime = other.waitTime;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
        return *this;
    }
    
    void wait()
    {
        if (!timer.isValid())
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mTimer);
            timer.start();
        }
        interrupted = false;
        timeout = false;
        std::unique_lock<std::mutex> lock(mWait);
        cvWait.wait(
            lock,
            [this] () -> bool
            {
                return interrupted || timeout;
            }
        );
    }

    void interrupt()
    {
        if (!timer.isValid() || !timer.hasStarted())
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mTimer);
            timer.stop();
        }
        std::lock_guard<std::mutex> lock(mWait);
        interrupted = true;
        cvWait.notify_one();
    }

    bool isInterrupted()
    {
        if (!timer.isValid())
        {
            return false;
        }

        return interrupted;
    }
};

#endif // H_WAIT_TIMER