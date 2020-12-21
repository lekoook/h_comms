#include "ReqMachine.hpp"

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
        currentState->recvAck(ackMsg, src);
    }
}

void ReqMachine::recvData(DataMsg& dataMsg, std::string src)
{
    if (currentState)
    {
        currentState->recvData(dataMsg, src);
    }
}