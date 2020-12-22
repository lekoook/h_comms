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
    std::cout << "RESPONDER: START" << std::endl;
    setState(machine, new SendAckReqRespState());
}

//// StartRespState END ////

//// SendAckReqRespState ////

SendAckReqRespState::SendAckReqRespState() : RespState(RespStateT::SendAckReq) {}

SendAckReqRespState::~SendAckReqRespState() {}

void SendAckReqRespState::run(RespMachine& machine)
{
    std::cout << "RESPONDER: PREPARE" << std::endl;
    // TODO: Request for data.
    uint64_t mockTs = 1234;
    std::vector<uint8_t> mockData = {5, 6, 7, 8};
    machine.dataToSend = DataMsg(machine.respSequence, machine.respEntryId, mockTs, mockData);

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
    std::cout << "RESPONDER: SEND DATA" << std::endl;
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
    std::cout << "RESPONDER: WAIT ACK DATA" << std::endl;
    machine._setWaitParams(machine.respSequence, machine.respEntryId);
    std::unique_lock<std::mutex> lock(machine.mGotMsg);
    bool ackSuccess = machine.cvGotMsg.wait_for(
        lock,
        std::chrono::milliseconds(5000),
        [&machine] () -> bool
            {
                return machine.gotMsg;
            }
    );

    if (ackSuccess || machine.sendTries >= machine.MAX_SEND_TRIES)
    {
        setState(machine, new DestructRespState());
    }
    else
    {
        setState(machine, new SendDataRespState());
    }
}

void WaitAckDataRespState::recvAck(RespMachine& machine, AckMsg& ackMsg, std::string src)
{
    if (machine._checkWaitParams(ackMsg.ackSequence, ackMsg.ackEntryId))
    {
        std::lock_guard<std::mutex> lock(machine.mGotMsg);
        machine.gotMsg = true;
        machine.cvGotMsg.notify_one();
    }
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
