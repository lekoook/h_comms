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
    std::cout << "REQUESTING FOR: " << m.reqSequence << " , " << m.reqEntryId << std::endl;
    
    if (machine.transmitter->transmit(machine.reqTarget, m))
    {
        machine._setWaitAckParams(machine.reqSequence, machine.reqEntryId);
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
    std::cout << "STATE: Wait ACK REQ" << std::endl;
    std::unique_lock<std::mutex> lock(machine.mGotAck);
    bool ackSuccess = machine.cvGotAck.wait_for(
        lock,
        std::chrono::milliseconds(1000),
        [&machine] () -> bool
            {
                return machine.gotAck;
            }
    );

    if (ackSuccess)
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
    if (machine._checkWaitAckParams(ackMsg.ackSequence, ackMsg.ackEntryId))
    {
        std::lock_guard<std::mutex> lock(machine.mGotAck);
        machine.gotAck = true;
        machine.cvGotAck.notify_one();
    }
}

//// WaitAckReqState END ////

//// WaitDataReqState ////

WaitDataReqState::WaitDataReqState() : ReqState(ReqStateT::WaitData) {}

WaitDataReqState::~WaitDataReqState() {}

void WaitDataReqState::run(ReqMachine& machine)
{
    std::cout << "STATE: Wait Data" << std::endl;
    setState(machine, new DestructReqState());
}

//// WaitDataReqState END ////

//// SendAckReqState ////

SendAckReqState::SendAckReqState() : ReqState(ReqStateT::SendAckData) {}

SendAckReqState::~SendAckReqState() {}

void SendAckReqState::run(ReqMachine& machine)
{
}

//// SendAckReqState END ////

//// RequeueReqState ////

RequeueReqState::RequeueReqState() : ReqState(ReqStateT::RequeueReq) {}

RequeueReqState::~RequeueReqState() {}

void RequeueReqState::run(ReqMachine& machine)
{
    std::cout << "STATE: Requeue REQ" << std::endl;
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
