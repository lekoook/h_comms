#ifndef MSG_OBSERVE_H
#define MSG_OBSERVE_H

#include <stdint.h>
#include "MsgTypes.hpp"

class IMsgObserver
{
public:
    virtual void update(uint8_t* msg, uint32_t msgLen, uint16_t src) = 0;
};

class IMsgObservable
{
public:
    /**
     * @brief Subscribes to the observable object for AODV messages.
     * 
     * @param observer Object that wants to observe this object for AODV messages.
     * @param msgType Type of AODV message to subscribe to.
     * @param all If this is true, @param msgType parameter will be ignored and all types will be subscribed to. Default is false.
     */
    virtual void subscribe(IMsgObserver* observer, aodv_msgs::MsgTypes msgType, bool all=false) = 0;
    
    /**
     * @brief Unsubscribes to the observable object for AODV messages.
     * @details When unsubscribing, @param msgType must be of the same type used when subscribing earlier. 
     * Likewise, @param msgType will be ignored and all types will be unsubscribed from if @param all is true.
     * 
     * @param observer Object that was observing this object for AODV messages.
     * @param msgType Type of AODV message to subscribe from.
     * @param all If this is true, @param msgType parameter will be ignored and all types will be unsubscribed from. Default is false.
     */
    virtual void unsubscribe(IMsgObserver* observer, aodv_msgs::MsgTypes msgType, bool all=false) = 0;

private:
    virtual void notify(uint8_t* msg, uint32_t msgLen, uint16_t src) = 0;
};

#endif // MSG_OBSERVE_H