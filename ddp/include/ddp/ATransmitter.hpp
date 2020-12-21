#ifndef H_ATRANSMITTER
#define H_ATRANSMITTER

#include <string>
#include <vector>
#include <functional>
#include "messages/BaseMsg.hpp"

/**
 * @brief Interface for transmission of messages.
 * 
 */
class ATransmitter
{
public:
    /**
     * @brief Interface to transmission messages.
     * 
     * @param dest Intended destination of message.
     * @param msg Message to transmit.
     * @return true If calling transmission service was successful.
     * @return false If calling transmission service was unsuccessful.
     */
    virtual bool transmit(std::string dest, BaseMsg& msg) = 0;
};

#endif // H_ATRANSMITTER