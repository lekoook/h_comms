#ifndef H_RESPONDER
#define H_RESPONDER

#include <thread>
#include "ALifeEntity.hpp"
#include "RespMachine.hpp"
#include "RespStates.hpp"

/**
 * @brief Represents the life time of a Responder in a data exchange.
 * 
 */
class Responder  : public ALifeEntity
{
private:
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
        RespMachine rsm(respSequence, respEntryId, respTarget, transmitter);
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
    }

public:
    /**
     * @brief Construct a new Responder object.
     * 
     * @param respSequence Sequence number of this response.
     * @param respEntryId Entry ID this response is made for.
     * @param respTarget Intended target address this response is to be made to.
     * @param transmitter Interface used to send messages. 
     */
    Responder(uint32_t respSequence, uint16_t respEntryId, std::string respTarget, ATransmitter* transmitter)
        : ALifeEntity(), respSequence(respSequence), respEntryId(respEntryId), respTarget(respTarget), 
            transmitter(transmitter)
    {
        _rsm = nullptr;
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
            std::cout << "RESPONDER GOT ACK" << std::endl;
            _rsm->recvAck(ackMsg, src);
        }
    }
};

#endif // H_RESPONDER