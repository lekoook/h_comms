#ifndef H_AODV_BASE_MSG
#define H_AODV_BASE_MSG

#include <cstdint>
#include <vector>

namespace aodv
{

enum MsgType
{
    RReq = 1,
    RRep = 2,
    RRer = 3,
    RRepAck = 4
};

class BaseMsg
{
public:
    MsgType msgType;
    BaseMsg(MsgType msgType) : msgType(msgType) {}
    virtual std::vector<uint8_t> serialize() = 0;
    virtual void deserialize(std::vector<uint8_t> msg) = 0;
};

}

#endif // H_BASE_MSG