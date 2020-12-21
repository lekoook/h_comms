#include "RespStates.hpp"
#include <iostream>

//// RespState ////

RespState::RespState(RespStateT state) : state(state) {}

RespState::~RespState() {}

void RespState::recvAck(AckMsg& ackMsg, std::string src) {}

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
}

//// StartRespState END ////

//// PrepRespState ////

PrepRespState::PrepRespState() : RespState(RespStateT::Prep) {}

PrepRespState::~PrepRespState() {}

void PrepRespState::run(RespMachine& machine)
{
}

//// PrepRespState END ////

//// SendDataRespState ////

SendDataRespState::SendDataRespState() : RespState(RespStateT::SendData) {}

SendDataRespState::~SendDataRespState() {}

void SendDataRespState::run(RespMachine& machine)
{
}

//// SendDataRespState END ////

//// WaitAckDataRespState ////

WaitAckDataRespState::WaitAckDataRespState() : RespState(RespStateT::WaitAckData) {}

WaitAckDataRespState::~WaitAckDataRespState() {}

void WaitAckDataRespState::run(RespMachine& machine)
{
}

//// WaitAckDataRespState END ////

//// NoDataRespState ////

NoDataRespState::NoDataRespState() : RespState(RespStateT::NoData) {}

NoDataRespState::~NoDataRespState() {}

void NoDataRespState::run(RespMachine& machine)
{
}

//// NoDataRespState END ////

//// DestructRespState ////

DestructRespState::DestructRespState() : RespState(RespStateT::Destruct) {}

DestructRespState::~DestructRespState() {}

void DestructRespState::run(RespMachine& machine)
{
    machine.isDestructed = true;
}

//// DestructRespState END ////
