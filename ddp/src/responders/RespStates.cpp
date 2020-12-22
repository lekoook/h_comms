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
    std::cout << "RESPONDER: START" << std::endl;
    setState(machine, new PrepRespState());
}

//// StartRespState END ////

//// PrepRespState ////

PrepRespState::PrepRespState() : RespState(RespStateT::Prep) {}

PrepRespState::~PrepRespState() {}

void PrepRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: PREPARE" << std::endl;
    AckMsg msg(machine.respSequence, machine.respEntryId);
    machine.transmitter->transmit(machine.respTarget, msg);
    setState(machine, new SendDataRespState());
}

//// PrepRespState END ////

//// SendDataRespState ////

SendDataRespState::SendDataRespState() : RespState(RespStateT::SendData) {}

SendDataRespState::~SendDataRespState() {}

void SendDataRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: SEND DATA" << std::endl;
    // TODO: Request for data.
    uint64_t mockTs = 1234;
    std::vector<uint8_t> mockData = {5, 6, 7, 8};
    DataMsg msg(machine.respSequence, machine.respEntryId, mockTs, mockData);
    machine.transmitter->transmit(machine.respTarget, msg);
    setState(machine, new WaitAckDataRespState());
}

//// SendDataRespState END ////

//// WaitAckDataRespState ////

WaitAckDataRespState::WaitAckDataRespState() : RespState(RespStateT::WaitAckData) {}

WaitAckDataRespState::~WaitAckDataRespState() {}

void WaitAckDataRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: WAIT ACK DATA" << std::endl;
    setState(machine, new DestructRespState());
}

//// WaitAckDataRespState END ////

//// NoDataRespState ////

NoDataRespState::NoDataRespState() : RespState(RespStateT::NoData) {}

NoDataRespState::~NoDataRespState() {}

void NoDataRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: NO DATA" << std::endl;
}

//// NoDataRespState END ////

//// DestructRespState ////

DestructRespState::DestructRespState() : RespState(RespStateT::Destruct) {}

DestructRespState::~DestructRespState() {}

void DestructRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: DESTRUCT" << std::endl;
    machine.isDestructed = true;
}

//// DestructRespState END ////
