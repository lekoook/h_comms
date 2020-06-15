#ifndef SOCKETTESTER_H
#define SOCKETTESTER_H

#include <iostream>
#include <stdint.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>

class SocketTransceiver
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
    SocketTransceiver(uint16_t port)
    {
        this->port = port;
    }

    // TODO: Create dtor.

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
    }

    /**
     * @brief Listens to messages forever. Blocking call.
     * 
     * TODO: Use thread to make this non blocking.
     * TODO: Use some observer pattern to design some sort of callbacks for subscribers subscribing to certain source port.
     * 
     */
    void listen()
    {
        struct sockaddr_in client;
        while(1) 
        {
            int len, n;
            len = sizeof(client);

            printf("waiting on port %d\n", port);
            n = recvfrom(fd, buffer, MAX_LEN, 0, (struct sockaddr *)&client, (socklen_t *)&len);
            printf("received %d bytes\n", n);

            // Find out who its from.
            char ip[INET_ADDRSTRLEN];
            uint16_t p;
            in_addr ipAddr = client.sin_addr;
            inet_ntop(AF_INET, &ipAddr, ip, INET_ADDRSTRLEN);
            p = htons(client.sin_port);
            printf ("from %s:%d\n", ip, p);

            // Print data.
            if (n > 0) {
                buffer[n] = 0;
                printf("received message: \"%s\"\n", buffer);
            }
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

private:
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
};

#endif // SOCKETTESTER_H