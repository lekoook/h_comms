#include "RespStates.hpp"
#include <iostream>

//// RespState ////

RespState::RespState(RespStateT state) : state(state) {}

RespState::~RespState() {}

void RespState::recvAck(RespMachine& machine, AckMsg& ackMsg, std::string src) {}

void RespState::setState(RespMachine& machine, RespState* newState)
{
    machine.nextState = newState;
}

//// RespState END ////

//// StartRespState ////

StartRespState::StartRespState() : RespState(RespStateT::Start) {}

StartRespState::~StartRespState() {}

void StartRespState::run(RespMachine& machine)
{
    setState(machine, new SendAckReqRespState());
}

//// StartRespState END ////

//// SendAckReqRespState ////

SendAckReqRespState::SendAckReqRespState() : RespState(RespStateT::SendAckReq) {}

SendAckReqRespState::~SendAckReqRespState() {}

void SendAckReqRespState::run(RespMachine& machine)
{
    AckMsg msg(machine.respSequence, machine.respEntryId);
    machine.transmitter->transmit(machine.respTarget, msg);
    setState(machine, new SendDataRespState());
}

//// SendAckReqRespState END ////

//// SendDataRespState ////

SendDataRespState::SendDataRespState() : RespState(RespStateT::SendData) {}

SendDataRespState::~SendDataRespState() {}

void SendDataRespState::run(RespMachine& machine)
{
    machine.transmitter->transmit(machine.respTarget, machine.dataToSend);
    machine.sendTries++;
    setState(machine, new WaitAckDataRespState());
}

//// SendDataRespState END ////

//// WaitAckDataRespState ////

WaitAckDataRespState::WaitAckDataRespState() : RespState(RespStateT::WaitAckData) {}

WaitAckDataRespState::~WaitAckDataRespState() {}

void WaitAckDataRespState::run(RespMachine& machine)
{
    machine._setWaitParams(machine.respSequence, machine.respEntryId);
    machine.waitTimer.wait();

    if (machine.waitTimer.isInterrupted() || machine.sendTries >= machine.MAX_SEND_TRIES)
    {
        setState(machine, new DestructRespState());
    }
    else
    {
        ROS_WARN("Responder to sequence %u for entry %u from %s TIMEOUT", 
            machine.respSequence, machine.respEntryId, machine.respTarget.c_str());
        setState(machine, new SendDataRespState());
    }
}

void WaitAckDataRespState::recvAck(RespMachine& machine, AckMsg& ackMsg, std::string src)
{
    if (machine._checkWaitParams(ackMsg.ackSequence, ackMsg.ackEntryId))
    {
        machine.waitTimer.interrupt();
    }
}

//// WaitAckDataRespState END ////

//// DestructRespState ////

DestructRespState::DestructRespState() : RespState(RespStateT::Destruct) {}

DestructRespState::~DestructRespState() {}

void DestructRespState::run(RespMachine& machine)
{
    machine.isDestructed = true;
}

//// DestructRespState END ////