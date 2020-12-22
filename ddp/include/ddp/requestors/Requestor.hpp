#ifndef H_REQUESTOR
#define H_REQUESTOR

#include <thread>
#include "ALifeEntity.hpp"
#include "ReqMachine.hpp"
#include "ReqStates.hpp"

class Requestor : public ALifeEntity
{
private:
    /**
     * @brief Sequence number of this request.
     * 
     */
    uint32_t reqSequence;

    /**
     * @brief Entry ID this request is made for.
     * 
     */
    uint16_t reqEntryId;

    /**
     * @brief Intended target address this request is to be made to.
     * 
     */
    std::string reqTarget;

    /**
     * @brief Interface used to send messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Pointer to a Requestor state machine.
     * 
     */
    ReqMachine* _rsm;

    /**
     * @brief Executes the lifetime of the Requestor state machine.
     * 
     */
    void _life()
    {
        ReqMachine rsm(reqSequence, reqEntryId, reqTarget, transmitter);
        _rsm = &rsm;
        while(lifeRunning.load())
        {
            rsm.run();
            rsm.checkTransit();
            if (rsm.isDestructed)
            {
                lifeRunning.store(false);
            }
        }
    }

public:
    /**
     * @brief Construct a new Requestor object.
     * 
     * @param reqSequence Sequence number of this request.
     * @param reqEntryId Entry ID this request is made for.
     * @param reqTarget Intended target address this request is to be made to.
     * @param transmitter Interface used to send messages. 
     */
    Requestor(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, ATransmitter* transmitter)
        : ALifeEntity(), reqSequence(reqSequence), reqEntryId(reqEntryId), reqTarget(reqTarget), 
            transmitter(transmitter) {}

    /**
     * @brief Notifies the Requestor of a received ACK message.
     * 
     * @param ackMsg ACK message received.
     * @param src Source address of this message.
     */
    void recvAck(AckMsg& ackMsg, std::string src)
    {
        _rsm->recvAck(ackMsg, src);
    }

    /**
     * @brief Notifies the Requestor of a received DATA message.
     * 
     * @param dataMsg DATA message received.
     * @param src Source address of this message.
     */
    void recvData(DataMsg& dataMsg, std::string src)
    {
        _rsm->recvData(dataMsg, src);
    }
};

#endif // H_REQUESTOR