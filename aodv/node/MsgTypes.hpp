#ifndef MSG_TYPES_H
#define MSG_TYPES_H

namespace aodv_msgs
{
    // TODO: Should we use 'All' and 'Unknown'?
    enum class MsgTypes
    {
        All,
        Rreq,
        Rrep,
        Rerr,
        RrepAck,
        Unknown
    };
};

#endif // MSG_TYPES_H