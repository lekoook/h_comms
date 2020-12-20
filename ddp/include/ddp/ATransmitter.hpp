#ifndef H_ATRANSMITTER
#define H_ATRANSMITTER

#include <string>
#include <vector>
#include <functional>
#include "messages/BaseMsg.hpp"

class ATransmitter
{
public:
    virtual bool transmit(std::string dest, BaseMsg& msg) = 0;
};

#endif // H_ATRANSMITTER