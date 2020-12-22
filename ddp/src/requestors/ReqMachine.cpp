#include "ReqMachine.hpp"

void ReqMachine::_setWaitParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    gotMsg = false;
    std::lock_guard<std::mutex> lock(mWaitParams);
    this->waitSeq = waitSeq;
    this->waitEntryId = waitEntryId;
}

bool ReqMachine::_checkWaitParams(uint32_t waitSeq, uint16_t waitEntryId)
{
    std::lock_guard<std::mutex> lock(mWaitParams);
    return (waitSeq == this->waitSeq && waitEntryId == this->waitEntryId);
}

bool ReqMachine::hasEnded()
{
    return isDestructed.load();
}

ReqMachine::ReqMachine(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, ATransmitter* transmitter) 
    : reqSequence(reqSequence), 
    reqEntryId(reqEntryId), 
    reqTarget(reqTarget), 
    currentState(new StartReqState()), 
    transmitter(transmitter)
{
    receivedData.store(false);
    isDestructed.store(false);
}

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

bool ReqMachine::hasReceived()
{
    return receivedData.load();
}