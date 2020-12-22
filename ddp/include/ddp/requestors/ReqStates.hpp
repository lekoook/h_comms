#ifndef H_REQ_STATES
#define H_REQ_STATES

#include "ReqMachine.hpp"
#include "messages/AckMsg.hpp"
#include "messages/DataMsg.hpp"
#include "messages/ReqMsg.hpp"

// Forward declaration of required class.
class ReqMachine;

/**
 * @brief Represents the base state in Requestor's state machine.
 * 
 */
class ReqState
{
public:
    /**
     * @brief Enumeration of all possible states.
     * 
     */
    enum ReqStateT
    {
        Start = 0,
        WaitAckReq = 1,
        WaitData = 2,
        SendAckData = 3,
        RequeueReq = 4,
        Destruct = 5
    };

    /**
     * @brief Indicates which specific state this state object is.
     * 
     */
    const ReqStateT state;

    /**
     * @brief Construct a new Req State object.
     * 
     * @param state The specific state of this state object.
     */
    ReqState(ReqStateT state);

    /**
     * @brief Destroy the Req State object.
     * 
     */
    virtual ~ReqState();

    //// External Events ////

    /**
     * @brief Notifies this state of a received ACK message.
     * 
     * @param machine State machine this state is exisitng under.
     * @param ackMsg ACK message received.
     * @param src Source address of this message.
     */
    virtual void recvAck(ReqMachine& machine, AckMsg& ackMsg, std::string src);

    /**
     * @brief Notifies this state of a received DATA message.
     * 
     * @param machine State machine this state is exisitng under.
     * @param dataMsg DATA message received.
     * @param src Source address of this message.
     */
    virtual void recvData(ReqMachine& machine, DataMsg& dataMsg, std::string src);

    //// External Events END ////

    //// Internal Events ////

    /**
     * @brief Main execution of the state.
     * 
     * @param machine State machine this state is exisitng under.
     */
    virtual void run(ReqMachine& machine) = 0;

    //// Internal Events END ////

protected:
    /**
     * @brief Sets the new state of the state machine.
     * 
     * @param machine The state machine this state is existing under.
     * @param newState New state to transition to.
     */
    void setState(ReqMachine& machine, ReqState* newState);
};

/**
 * @brief Represents the Start state in Requestor's state machine.
 * 
 */
class StartReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Start Req State object.
     * 
     */
    StartReqState();

    /**
     * @brief Destroy the Start Req State object.
     * 
     */
    virtual ~StartReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);
};

class WaitAckReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Wait Ack Req State object.
     * 
     */
    WaitAckReqState();

    /**
     * @brief Destroy the Wait Ack Req State object.
     * 
     */
    virtual ~WaitAckReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);

    /**
     * @brief Notify the state that an ACK has arrived.
     * 
     * @param machine The state machine this state is existing under.
     * @param ackMsg ACK message arrived.
     * @param src Source address of the ACK message.
     */
    virtual void recvAck(ReqMachine& machine, AckMsg& ackMsg, std::string src);
};

class WaitDataReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Wait Data Req State object.
     * 
     */
    WaitDataReqState();

    /**
     * @brief Destroy the Wait Data Req State object.
     * 
     */
    virtual ~WaitDataReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);

    /**
     * @brief Notify the state that an DATA has arrived.
     * 
     * @param machine The state machine this state is existing under.
     * @param dataMsg DATA message arrived.
     * @param src Source address of the DATA message.
     */
    virtual void recvData(ReqMachine& machine, DataMsg& dataMsg, std::string src);
};

class SendAckReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Send Ack Req State object.
     * 
     */
    SendAckReqState();

    /**
     * @brief Destroy the Send Ack Req State object.
     * 
     */
    virtual ~SendAckReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);
};

class RequeueReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Requeue Req State object.
     * 
     */
    RequeueReqState();

    /**
     * @brief Destroy the Requeue Req State object.
     * 
     */
    virtual ~RequeueReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);
};

class DestructReqState : public ReqState
{
public:
    /**
     * @brief Construct a new Destruct Req State object.
     * 
     */
    DestructReqState();

    /**
     * @brief Destroy the Destruct Req State object.
     * 
     */
    virtual ~DestructReqState();

    /**
     * @brief Executes the main sequence in this state.
     * 
     * @param machine The state machine this state is existing under.
     */
    virtual void run(ReqMachine& machine);
};

#endif // H_REQ_STATES