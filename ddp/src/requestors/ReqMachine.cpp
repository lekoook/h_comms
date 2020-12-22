#include "ReqMachine.hpp"

void ReqMachine::_setWaitAckParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    std::lock_guard<std::mutex> lock(mWaitAckParams);
    waitAckSeq = waitSeq;
    waitAckEntryId = waitEntryId;
}

bool ReqMachine::_checkWaitAckParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    std::lock_guard<std::mutex> lock(mWaitAckParams);
    return (waitSeq == waitAckSeq && waitEntryId == waitAckEntryId);
}

ReqMachine::ReqMachine(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, ATransmitter* transmitter) 
    : reqSequence(reqSequence), 
    reqEntryId(reqEntryId), 
    reqTarget(reqTarget), 
    currentState(new StartReqState()), 
    transmitter(transmitter)
{}

ReqMachine::~ReqMachine()
{
    delete currentState;
}

void ReqMachine::run()
{
    if (currentState)
    {
        currentState->run(*this);
    }
}

void ReqMachine::checkTransit()
{
    if (nextState)
    {
        delete currentState;
        currentState = nextState;
        nextState = nullptr;
    }
}

void ReqMachine::recvAck(AckMsg& ackMsg, std::string src)
{
    if (currentState)
    {
        currentState->recvAck(*this, ackMsg, src);
    }
}

void ReqMachine::recvData(DataMsg& dataMsg, std::string src)
{
    if (currentState)
    {
        currentState->recvData(*this, dataMsg, src);
    }
}