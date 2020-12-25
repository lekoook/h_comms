#ifndef H_RESPONDER
#define H_RESPONDER

#include <thread>
#include "ALifeEntity.hpp"
#include "RespMachine.hpp"
#include "RespStates.hpp"
#include "ADataAccessor.hpp"
#include "ARespManager.hpp"

/**
 * @brief Represents the life time of a Responder in a data exchange.
 * 
 */
class Responder  : public ALifeEntity
{
private:
    /**
     * @brief Source address of the Requestor to which this Responder is responding to.
     * 
     */
    std::string reqSrc;

    /**
     * @brief Sequence number for response.
     * 
     */
    uint32_t respSequence;

    /**
     * @brief Entry ID for this response.
     * 
     */
    uint16_t respEntryId;

    /**
     * @brief Intended target address for this response.
     * 
     */
    std::string respTarget;

    /**
     * @brief Interface used to send messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Interface used to access data in database.
     * 
     */
    ADataAccessor* dataAccessor;

    /**
     * @brief Interface of Responders manager that owns this Responder.
     * 
     */
    ARespManager* respManager;

    /**
     * @brief Pointer to a Responder state machine.
     * 
     */
    RespMachine* _rsm;

    /**
     * @brief Mutex to protect the pointer to Responder state machine.
     * 
     */
    std::mutex mRsm;

    /**
     * @brief Executes the lifetime of the Responder state machine.
     * 
     */
    void _life()
    {
        DataMsg msg;
        msg.reqSequence = respSequence;
        msg.entryId = respEntryId;
        dataAccessor->pullData(respEntryId, msg.timestamp, msg.data);
        RespMachine rsm(respSequence, respEntryId, respTarget, msg, transmitter);
        {
            std::lock_guard<std::mutex> lock(mRsm);
            _rsm = &rsm;
        }
        while(lifeRunning.load())
        {
            rsm.run();
            rsm.checkTransit();
            if (rsm.hasEnded())
            {
                lifeRunning.store(false);
            }
        }

        std::lock_guard<std::mutex> lock(mRsm);
        _rsm = nullptr;
        respManager->removeResp(reqSrc, respSequence);
    }

public:
    /**
     * @brief Construct a new Responder object.
     * 
     * @param reqSrc Source address of the Requestor to which this Responder is responding to.
     * @param respSequence Sequence number of this response.
     * @param respEntryId Entry ID this response is made for.
     * @param respTarget Intended target address this response is to be made to.
     * @param transmitter Interface used to send messages. 
     * @param dataAccessor Interface used to access data in database.
     * @param respManager Interface of Responders manager that owns this Responder.
     */
    Responder(std::string reqSrc, uint32_t respSequence, uint16_t respEntryId, std::string respTarget, 
        ATransmitter* transmitter, ADataAccessor* dataAccessor, ARespManager* respManager)
        : ALifeEntity(), reqSrc(reqSrc), respSequence(respSequence), respEntryId(respEntryId), respTarget(respTarget), 
            transmitter(transmitter), dataAccessor(dataAccessor), respManager(respManager), _rsm(nullptr) 
    {
        lifeRunning.store(true);
        lifeTh = std::thread(&Responder::_life, this);
    }

    /**
     * @brief Notifies the Responder of a received ACK message.
     * 
     * @param ackMsg ACK message received.
     * @param src Source address of this message.
     */
    void recvAck(AckMsg& ackMsg, std::string src)
    {
        std::lock_guard<std::mutex> lock(mRsm);
        if (_rsm)
        {
            _rsm->recvAck(ackMsg, src);
        }
    }
};

#endif // H_RESPONDER