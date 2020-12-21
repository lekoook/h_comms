#include "ReqStates.hpp"

//// ReqState ////

ReqState::ReqState(ReqStateT state) : state(state) {}

ReqState::~ReqState() {}

void ReqState::recvAck(AckMsg& ackMsg, std::string src) {}

void ReqState::recvData(DataMsg& dataMsg, std::string src) {}

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
    AckMsg m(machine.reqSequence, machine.reqEntryId);
    machine.transmitter->transmit(machine.reqTarget, m);
    setState(machine, new DestructReqState());
}

//// StartReqState END ////

//// WaitAckReqState ////

WaitAckReqState::WaitAckReqState() : ReqState(ReqStateT::WaitAckReq) {}

WaitAckReqState::~WaitAckReqState() {}

void WaitAckReqState::run(ReqMachine& machine)
{
}

//// WaitAckReqState END ////

//// WaitDataReqState ////

WaitDataReqState::WaitDataReqState() : ReqState(ReqStateT::WaitData) {}

WaitDataReqState::~WaitDataReqState() {}

void WaitDataReqState::run(ReqMachine& machine)
{
}

//// WaitDataReqState END ////

//// SendAckReqState ////

SendAckReqState::SendAckReqState() : ReqState(ReqStateT::WaitData) {}

SendAckReqState::~SendAckReqState() {}

void SendAckReqState::run(ReqMachine& machine)
{
}

//// SendAckReqState END ////

//// RequeueReqState ////

RequeueReqState::RequeueReqState() : ReqState(ReqStateT::WaitData) {}

RequeueReqState::~RequeueReqState() {}

void RequeueReqState::run(ReqMachine& machine)
{
}

//// RequeueReqState END ////

//// DestructReqState ////

DestructReqState::DestructReqState() : ReqState(ReqStateT::WaitData) {}

DestructReqState::~DestructReqState() {}

void DestructReqState::run(ReqMachine& machine)
{
    machine.isDestructed = true;
}

//// DestructReqState END ////
