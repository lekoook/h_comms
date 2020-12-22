#include "RespMachine.hpp"

void RespMachine::_setWaitParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    gotMsg = false;
    std::lock_guard<std::mutex> lock(mWaitParams);
    this->waitSeq = waitSeq;
    this->waitEntryId = waitEntryId;
}

bool RespMachine::_checkWaitParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    std::lock_guard<std::mutex> lock(mWaitParams);
    return (waitSeq == this->waitSeq && waitEntryId == this->waitEntryId);
}

bool RespMachine::hasEnded()
{
    isDestructed.load();
}

RespMachine::RespMachine(uint32_t respSequence, uint16_t respEntryId, std::string respTarget, ATransmitter* transmitter) 
    : respSequence(respSequence), 
    respEntryId(respEntryId), 
    respTarget(respTarget), 
    currentState(new StartRespState()), 
    transmitter(transmitter)
{
    isDestructed.store(false);
}

RespMachine::~RespMachine()
{
    delete currentState;
}

void RespMachine::run()
{
    if (currentState)
    {
        currentState->run(*this);
    }
}

void RespMachine::checkTransit()
{
    if (nextState)
    {
        delete currentState;
        currentState = nextState;
        nextState = nullptr;
    }
}

void RespMachine::recvAck(AckMsg& ackMsg, std::string src)
{
    if (currentState)
    {
        currentState->recvAck(*this, ackMsg, src);
    }
}
