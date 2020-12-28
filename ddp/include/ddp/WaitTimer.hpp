#ifndef H_WAIT_TIMER
#define H_WAIT_TIMER

#include "ros/ros.h"
#include <condition_variable>

/**
 * @brief Timer that allows caller to wait for a specific amount of simulation time (seconds) or interrupt the timer
 * before it elapses.
 * 
 */
class WaitTimer
{
private:
    /**
     * @brief ROS timer used to wait in simulation time.
     * 
     */
    ros::Timer timer;

    /**
     * @brief Mutex to protect the timer.
     * 
     */
    std::mutex mTimer;

    /**
     * @brief Used to signal the end of waiting either by elapse of timer or interrupt.
     * 
     */
    std::condition_variable cvWait;

    /**
     * @brief Mutex used for signalling.
     * 
     */
    std::mutex mWait;
    
    /**
     * @brief Flag to indicate if the timer was interrupted.
     * 
     */
    std::atomic<bool> interrupted;

    /**
     * @brief Flag to indicate if the timer has elapsed.
     * 
     */
    std::atomic<bool> timeout;
    
    /**
     * @brief Callback for when the timer elapses.
     * 
     * @param event 
     */
    void timerCallback(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mWait);
        timeout = true;
        cvWait.notify_one();
    }
    
public:
    /**
     * @brief Amount of time (seconds) to wait for.
     * 
     */
    double waitTime;

    /**
     * @brief Construct a new Wait Timer object.
     * 
     */
    WaitTimer() : waitTime(0) {}

    /**
     * @brief Construct a new Wait Timer object with a timer that will elapse after the specified waiting time.
     * 
     * @param waitTime Amount of time to wait for the timer to elapse.
     */
    WaitTimer(double waitTime) : waitTime(waitTime)
    {
        interrupted = false;
        timeout = false;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
    }

    /**
     * @brief Copy constructor.
     * 
     */
    WaitTimer(const WaitTimer& other)
    {
        interrupted = false;
        timeout = false;
        waitTime = other.waitTime;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
    }

    /**
     * @brief Move constructor.
     * 
     */
    WaitTimer(const WaitTimer&& other)
    {
        interrupted = false;
        timeout = false;
        waitTime = other.waitTime;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
    }

    /**
     * @brief Copy assignment.
     * 
     */
    WaitTimer& operator=(const WaitTimer& other)
    {
        interrupted = false;
        timeout = false;
        waitTime = other.waitTime;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
        return *this;
    }

    /**
     * @brief Move assignment.
     * 
     */
    WaitTimer& operator=(const WaitTimer&& other)
    {
        interrupted = false;
        timeout = false;
        waitTime = other.waitTime;
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(waitTime), &WaitTimer::timerCallback, this, true, false);
        return *this;
    }
    
    /**
     * @brief Begin waiting for the timer to elapse. Does nothing if the timer was not initialized or wait time is zero.
     * @details This is a blocking call. It will return when either the timer has elapsed or if it was interrupted.
     * 
     */
    void wait()
    {
        if (!timer.isValid() || waitTime == 0)
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

    /**
     * @brief Interrupts the timer so it will return from waiting if is already waiting.
     * 
     */
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

    /**
     * @brief Queries if the timer was interrupted since the last waiting call.
     * 
     * @return true If the timer was interrupted.
     * @return false If the timer was not interrupted and has elapsed.
     */
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