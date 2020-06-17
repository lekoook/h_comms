#ifndef SOCKET_TRANSCEIVER_H
#define SOCKET_TRANSCEIVER_H

#include <iostream>
#include <stdint.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <list>

#include "MsgObserve.hpp"
#include "MsgTypes.hpp"
#include "MsgPeeker.hpp"

/**
 * @brief Allows data to be sent and receives (on a separate thread) through a specified port.
 * 
 */
class SocketTransceiver : public IMsgObservable
{
public:
    /**
     * @brief Maximum length of a message.
     * 
     */
    static const uint16_t MAX_LEN = 1024;

    /**
     * @brief Construct a new Socket Transceiver object.
     * 
     * @param port Port to use in this socket.
     */
    SocketTransceiver(uint16_t port) : port(port)
    {
        if (!init())
            std::cerr << "Cannot initialize socket" << std::endl;
    }

    // TODO: Create dtor. Maybe not? So long creator calls stopListen() before destroying this object.

    /**
     * @brief Start listening to messages.
     * 
     * TODO: Use some observer pattern to design some sort of callbacks for subscribers subscribing to certain source port.
     * 
     */
    void listen()
    {
        if (thReceive.joinable())
            return; // A thread has already started previously, don't start a new one.

        listening = true;
        thReceive = std::thread(&SocketTransceiver::tReceive, this);
    }

    /**
     * @brief Stop listening to messages.
     * 
     */
    void stopListen()
    {
        if (thReceive.joinable())
        {
            listening = false;
            thReceive.join();
        }
    }

    /**
     * @brief Sends bytes data to a destination port.
     * 
     * @param data Bytes array containing the data to send.
     * @param len Byte length of data.
     * @param destPort Destination port to send this data to.
     * 
     * @return true if send is successful.
     * @return false otherwise.
     */
    bool send(uint8_t* data, uint16_t len, uint16_t destPort)
    {
        // Create socket address to the destination port.
        struct sockaddr_in dest;
        int destFd;
        if ((destFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            std::cerr << "Error getting to destination" << std::endl;
            return false;
        }
        memset(&dest, 0, sizeof(dest));
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = INADDR_ANY;
        dest.sin_port = htons(destPort);

        // Send data with our own file descriptor to the destination socket address.
        sendto(fd, data, len, MSG_CONFIRM, (const struct sockaddr *)&dest, sizeof(dest));
    }

    /// IMsgObservable implementations ///

    void subscribe(IMsgObserver* observer, aodv_msgs::MsgTypes msgType, bool all=false)
    {
        if (all)
        {
            std::lock_guard<std::mutex> g(mAll);
            allObservers.push_back(observer);
            return;
        }

        if (msgType == aodv_msgs::MsgTypes::Rreq)
        {
            std::lock_guard<std::mutex> g(mRreq);
            rreqObservers.push_back(observer);
        }
        else if (msgType == aodv_msgs::MsgTypes::Rrep)
        {
            std::lock_guard<std::mutex> g(mRrep);
            rrepObservers.push_back(observer);
        }
        else if (msgType == aodv_msgs::MsgTypes::Rrer)
        {
            std::lock_guard<std::mutex> g(mRrer);
            rrerObservers.push_back(observer);
        }
    }

    void unsubscribe(IMsgObserver* observer, aodv_msgs::MsgTypes msgType, bool all=false)
    {
        if (all)
        {
            std::lock_guard<std::mutex> g(mAll);
            allObservers.remove(observer);
            return;
        }

        if (msgType == aodv_msgs::MsgTypes::Rreq)
        {
            std::lock_guard<std::mutex> g(mRreq);
            rreqObservers.remove(observer);
        }
        else if (msgType == aodv_msgs::MsgTypes::Rrep)
        {
            std::lock_guard<std::mutex> g(mRrep);
            rrepObservers.remove(observer);
        }
        else if (msgType == aodv_msgs::MsgTypes::Rrer)
        {
            std::lock_guard<std::mutex> g(mRrer);
            rrerObservers.remove(observer);
        }
    }

private:
    /// Variable members ///

    /**
     * @brief File descriptor handle.
     * 
     */
    int fd;

    /**
     * @brief Port to use in this socket.
     * @details Because we are just sending within localhost, our port become our 'address'.
     * 
     */
    uint16_t port;

    /**
     * @brief Buffer to hold data. Has MAX_LEN max bytes.
     * 
     */
    uint8_t buffer[MAX_LEN];

    /**
     * @brief Struct to describe socket address.
     * 
     */
    struct sockaddr_in svrAddr;

    /**
     * @brief Thread that listens and receives messages.
     * 
     */
    std::thread thReceive;

    /**
     * @brief Flag used to check if we should continue listening for messages.
     * 
     */
    bool listening;

    std::list<IMsgObserver*> allObservers;
    std::list<IMsgObserver*> rreqObservers;
    std::list<IMsgObserver*> rrepObservers;
    std::list<IMsgObserver*> rrerObservers;

    std::mutex mAll;
    std::mutex mRreq;
    std::mutex mRrep;
    std::mutex mRrer;

    /// Method members ///

    /**
     * @brief Initialises the socket with a pre-defined default config. This MUST be called.
     * 
     * @return true if socket initialise and binding is successful.
     * @return false if initialize or binding failed.
     */
    bool init()
    {
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0)
        {
            std::cerr << "Cannot create socket" << std::endl;
            return false;
        }

        memset(&svrAddr, 0, sizeof(svrAddr));
        svrAddr.sin_family = AF_INET;
        svrAddr.sin_addr.s_addr = INADDR_ANY;
        svrAddr.sin_port = htons(port);

        if (bind(fd, (const struct sockaddr *)&svrAddr, sizeof(svrAddr)) < 0)
        {
            std::cerr << "Cannot bind server" << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief Attempts to receive data bytes from any source.
     * 
     * @param recvData Buffer to hold the received bytes.
     * @param dataLen Number of bytes received.
     * @param src Source of data.
     * @return true if at least 1 byte was received.
     * @return false if no bytes was received.
     */
    bool receive(uint8_t* recvData, uint32_t& dataLen, uint16_t& src)
    {
        // Receive bytes
        struct sockaddr_in source;
        uint32_t len = 0, bytes = 0;
        len = sizeof(source);

        // Check if there are data ready for our socket file descriptor before reading it.
        // We do this because recvfrom() is blocking until socket closes (although not guaranteed) or some data arrives.
        struct timeval tv = {0, 100000}; // 100ms
        fd_set fdSet;
        FD_ZERO(&fdSet);
        FD_SET(fd, &fdSet);
        if (select(fd+1, &fdSet, NULL, NULL, &tv) >= 0)
        {
            if (FD_ISSET(fd, &fdSet))
            {
                // There are data for us, read it.
                bytes = recvfrom(fd, buffer, MAX_LEN, 0, (struct sockaddr *)&source, (socklen_t *)&len);

                // Prepare return values.
                memcpy(recvData, buffer, bytes);
                dataLen = bytes;

                // Find out who its from.
                src = htons(source.sin_port);

                if (bytes > 0)
                    return true;
            }
        }
        return false;
    }

    /**
     * @brief Thread method to receive data.
     * 
     */
    void tReceive()
    {
        printf("listening on port %d\n", port);
        
        while(listening)
        {
            uint8_t buf[MAX_LEN];
            uint32_t len;
            uint16_t src;
            if (receive(buf, len, src))            
                notify(buf, len, src);
        }
    }

    /// IMsgObservable implementations ///
    
    void notify(uint8_t* msg, uint32_t msgLen, uint16_t src)
    {
        aodv_msgs::MsgTypes t = msg_peeker::peekType(msg);

        // Notify specific types.
        // We make a snapshot to prevent the list from being invalidated while notifying.
        std::list<IMsgObserver*> snaps;
        if (t == aodv_msgs::MsgTypes::Rreq)
        {
            std::lock_guard<std::mutex> g(mRreq);
            snaps = std::list<IMsgObserver*>(rreqObservers);
        }
        else if (t == aodv_msgs::MsgTypes::Rrep)
        {
            std::lock_guard<std::mutex> g(mRrep);
            snaps = std::list<IMsgObserver*>(rrepObservers);
        }
        else if (t == aodv_msgs::MsgTypes::Rrer)
        {
            std::lock_guard<std::mutex> g(mRrer);
            snaps = std::list<IMsgObserver*>(rrerObservers);
        }
        else
        {
            std::cerr << "Unknown message type, not notifying." << std::endl;
            return;
        }
        for (std::list<IMsgObserver*>::iterator it = snaps.begin(); it != snaps.end(); it++)
        {
            (*it)->update(msg, msgLen, src);
        }

        // Notify those who subscribed to all types.
        {
            std::lock_guard<std::mutex> g(mAll);
            snaps = std::list<IMsgObserver*>(allObservers);
        }
        for (std::list<IMsgObserver*>::iterator it = snaps.begin(); it != snaps.end(); it++)
        {
            (*it)->update(msg, msgLen, src);
        }
    }
};

#endif // SOCKET_TRANSCEIVER_H
