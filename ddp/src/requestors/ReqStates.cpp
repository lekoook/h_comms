#include "ReqStates.hpp"
#include <iostream>

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
    // std::cout << "REQUESTOR: REQUESTING - " << m.reqSequence << " , " << m.reqEntryId << std::endl;
    
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
    // std::cout << "REQUESTOR: WAIT ACK REQ" << std::endl;
    machine._setWaitParams(machine.reqSequence, machine.reqEntryId);
    machine.waitTimer.wait();

    if (machine.waitTimer.isInterrupted())
    {
        setState(machine, new WaitDataReqState());
    }
    else
    {
        std::cout << "Requestor " << machine.reqSequence << " to " << machine.reqTarget << " for " << machine.reqEntryId << " REQ timeout" << std::endl;
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
    // std::cout << "REQUESTOR: WAIT DATA" << std::endl;
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
    // std::cout << "REQUESTOR: SEND DATA ACK" << std::endl;
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
    std::cout << "REQUESTOR " << machine.reqSequence << ": REQUEUE REQ to " << machine.reqTarget << " for " << machine.reqEntryId << " DATA timeout" << std::endl;
    machine.needReqeue.store(true);
    setState(machine, new DestructReqState());
}

//// RequeueReqState END ////

//// DestructReqState ////

DestructReqState::DestructReqState() : ReqState(ReqStateT::Destruct) {}

DestructReqState::~DestructReqState() {}

void DestructReqState::run(ReqMachine& machine)
{
    // std::cout << "REQUESTOR: DESTRUCT" << std::endl;
    machine.isDestructed = true;
}

//// DestructReqState END ////
