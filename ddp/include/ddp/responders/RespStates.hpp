#ifndef H_RESP_STATES
#define H_RESP_STATES

#include "RespMachine.hpp"
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"

// Forward declaration of required class.
class RespMachine;

/**
 * @brief Represents the base state in Responder's state machine.
 * 
 */
class RespState
{
public:
    /**
     * @brief Enumeration of all possible states.
     * 
     */
    enum RespStateT
    {
        Start = 0,
        SendAckReq = 1,
        SendData = 2,
        WaitAckData = 3,
        Destruct = 4
    };

    /**
     * @brief Indicates which specific state this state object is.
     * 
     */
    const RespStateT state;

    /**
     * @brief Construct a new Resp State object.
     * 
     * @param state The specific state of this state object.
     */
    RespState(RespStateT state);

    /**
     * @brief Destroy the Resp State object.
     * 
     */
    virtual ~RespState();

    //// External Events ////

    /**
     * @brief Notifies this state of a received ACK message.
     * 
     * @param machine State machine this state is exisitng under.
     * @param ackMsg ACK message received.
     * @param src Source address of this message.
     */
    virtual void recvAck(RespMachine& machine, AckMsg& ackMsg, std::string src);

    //// External Events END ////

    //// Internal Events ////

    /**
     * @brief Main execution of the state.
     * 
     * @param machine State machine this state is exisitng under.
     */
    virtual void run(RespMachine& machine) = 0;

    //// Internal Events END ////

protected:
    /**
     * @brief Sets the new state of the state machine.
     * 
     * @param machine The state machine this state is existing under.
     * @param newState New state to transition to.
     */
    void setState(RespMachine& machine, RespState* newState);
};

/**
 * @brief Represents the Start state in Requestor's state machine.
 * 
 */
class StartRespState : public RespState
{
public:
    /**
     * @brief Construct a new Start Resp State object.
     * 
     */
    StartRespState();

    /**
     * @brief Destroy the Start Resp State object.
     * 
     */
    virtual ~StartRespState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(RespMachine& machine);
};

class SendAckReqRespState : public RespState
{
public:
    /**
     * @brief Construct a new Send Ack Req Resp State object.
     * 
     */
    SendAckReqRespState();

    /**
     * @brief Destroy the Send Ack Req Resp State object.
     * 
     */
    virtual ~SendAckReqRespState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(RespMachine& machine);
};

class SendDataRespState : public RespState
{
public:
    /**
     * @brief Construct a new Send Data Resp State object.
     * 
     */
    SendDataRespState();

    /**
     * @brief Destroy the Send Data Resp State object.
     * 
     */
    virtual ~SendDataRespState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(RespMachine& machine);
};

class WaitAckDataRespState : public RespState
{
public:
    /**
     * @brief Construct a new Wait Ack Data Resp State object.
     * 
     */
    WaitAckDataRespState();

    /**
     * @brief Destroy the Wait Ack Data Resp State object.
     * 
     */
    virtual ~WaitAckDataRespState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(RespMachine& machine);

    /**
     * @brief Notify the state that an ACK has arrived.
     * 
     * @param machine The state machine this state is existing under.
     * @param ackMsg ACK message arrived.
     * @param src Source address of the ACK message.
     */
    virtual void recvAck(RespMachine& machine, AckMsg& ackMsg, std::string src);
};

class DestructRespState : public RespState
{
public:
    /**
     * @brief Construct a new Destruct Resp State object.
     * 
     */
    DestructRespState();

    /**
     * @brief Destroy the Destruct Resp State object.
     * 
     */
    virtual ~DestructRespState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(RespMachine& machine);
};

#endif // H_RESP_STATES