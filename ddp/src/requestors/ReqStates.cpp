#include "ReqStates.hpp"

//// ReqState ////

ReqState::ReqState(ReqStateT state) : state(state) {}

ReqState::~ReqState() {}

void ReqState::recvAck(ReqMachine& machine, AckMsg& ackMsg, std::string src) {}

void ReqState::recvData(ReqMachine& machine, DataMsg& dataMsg, std::string src) {}

void ReqState::setState(ReqMachine& machine, ReqState* newState)
{
    machine.nextState = newState;
}

//// ReqState END ////

//// StartReqState ////

StartReqState::StartReqState() : ReqState(ReqStateT::Start) {}

StartReqState::~StartReqState() {}

void StartReqState::run(ReqMachine& machine)
{
    ReqMsg m(machine.reqSequence, machine.reqEntryId);
    
    if (machine.transmitter->transmit(machine.reqTarget, m))
    {
        setState(machine, new WaitAckReqState());
    }
    else
    {
        setState(machine, new RequeueReqState());
    }
}

//// StartReqState END ////

//// WaitAckReqState ////

WaitAckReqState::WaitAckReqState() : ReqState(ReqStateT::WaitAckReq) {}

WaitAckReqState::~WaitAckReqState() {}

void WaitAckReqState::run(ReqMachine& machine)
{
    machine._setWaitParams(machine.reqSequence, machine.reqEntryId);
    machine.waitTimer.wait();

    if (machine.waitTimer.isInterrupted())
    {
        setState(machine, new WaitDataReqState());
    }
    else
    {
        setState(machine, new RequeueReqState());
    }
}

void WaitAckReqState::recvAck(ReqMachine& machine, AckMsg& ackMsg, std::string src)
{
    if (machine._checkWaitParams(ackMsg.ackSequence, ackMsg.ackEntryId))
    {
        machine.waitTimer.interrupt();
    }
}

//// WaitAckReqState END ////

//// WaitDataReqState ////

WaitDataReqState::WaitDataReqState() : ReqState(ReqStateT::WaitData) {}

WaitDataReqState::~WaitDataReqState() {}

void WaitDataReqState::run(ReqMachine& machine)
{
    machine._setWaitParams(machine.reqSequence, machine.reqEntryId);
    machine.waitTimer.wait();

    if (machine.waitTimer.isInterrupted())
    {
        setState(machine, new SendAckReqState());
    }
    else
    {
        setState(machine, new RequeueReqState());
    }
}

void WaitDataReqState::recvData(ReqMachine& machine, DataMsg& dataMsg, std::string src)
{
    if (machine._checkWaitParams(dataMsg.reqSequence, dataMsg.entryId))
    {
        machine.waitTimer.interrupt();
        machine.dataReceived = dataMsg;
        machine.receivedData = true;
    }
}

//// WaitDataReqState END ////

//// SendAckReqState ////

SendAckReqState::SendAckReqState() : ReqState(ReqStateT::SendAckData) {}

SendAckReqState::~SendAckReqState() {}

void SendAckReqState::run(ReqMachine& machine)
{
    AckMsg msg(machine.reqSequence, machine.reqEntryId, false);
    machine.transmitter->transmit(machine.reqTarget, msg);
    setState(machine, new DestructReqState());
}

//// SendAckReqState END ////

//// RequeueReqState ////

RequeueReqState::RequeueReqState() : ReqState(ReqStateT::RequeueReq) {}

RequeueReqState::~RequeueReqState() {}

void RequeueReqState::run(ReqMachine& machine)
{
    machine.needReqeue.store(true);
    setState(machine, new DestructReqState());
}

//// RequeueReqState END ////

//// DestructReqState ////

DestructReqState::DestructReqState() : ReqState(ReqStateT::Destruct) {}

DestructReqState::~DestructReqState() {}

void DestructReqState::run(ReqMachine& machine)
{
    machine.isDestructed = true;
}

//// DestructReqState END ////
