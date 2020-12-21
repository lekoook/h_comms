#include "RespMachine.hpp"

RespMachine::RespMachine(uint32_t respSequence, uint16_t respEntryId, std::string respTarget, ATransmitter* transmitter) 
    : respSequence(respSequence), 
    respEntryId(respEntryId), 
    respTarget(respTarget), 
    currentState(new StartRespState()), 
    transmitter(transmitter)
{}

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
        currentState->recvAck(ackMsg, src);
    }
}
