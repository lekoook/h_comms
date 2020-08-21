#include "AckMsg.hpp"
#include "../utilities/serialisers.hpp"

#include <iostream>

namespace aodv
{
    AckMsg::AckMsg(uint32_t sequence, uint32_t segmentSequence) : seq(sequence), segSeq(segmentSequence)
    {}

    AckMsg::AckMsg(uint8_t ackMsg[AckMsgSize])
    {
        if (ackMsg[TypeIdx] != Type)
        {
            seq = 0;
            segSeq = 0;
            std::cerr << "Serialised message is not of Ack type." << std::endl;
        }
        else
        {
            seq = serialisers::getU32(&ackMsg[SeqIdx]);
            segSeq = serialisers::getU32(&ackMsg[SegSeqIdx]);
        }
    }

    void AckMsg::serialise(uint8_t ackMsg[AckMsgSize])
    {
        serialisers::copyU8(&ackMsg[TypeIdx], Type);
        serialisers::copyU32(&ackMsg[SeqIdx], seq);
        serialisers::copyU32(&ackMsg[SegSeqIdx], segSeq);
    }

    bool AckMsg::operator==(const AckMsg& ackMsg)
    {
        return seq == ackMsg.seq && segSeq == ackMsg.segSeq;
    }
}