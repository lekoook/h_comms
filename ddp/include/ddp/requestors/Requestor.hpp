#ifndef H_REQUESTOR
#define H_REQUESTOR

#include <thread>
#include "ALifeEntity.hpp"
#include "ReqMachine.hpp"
#include "ReqStates.hpp"
#include "ADataAccessor.hpp"
#include "AReqManager.hpp"

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
     * @brief Interface used to access data in database.
     * 
     */
    ADataAccessor* dataAccessor;

    /**
     * @brief Interface of Requestors manager that owns this Requestor.
     * 
     */
    AReqManager* reqManager;

    /**
     * @brief Pointer to a Requestor state machine.
     * 
     */
    ReqMachine* _rsm = nullptr;

    /**
     * @brief Mutex to protect pointer to Requestor.
     * 
     */
    std::mutex mRsm;

    /**
     * @brief Executes the lifetime of the Requestor state machine.
     * 
     */
    void _life()
    {
        ReqMachine rsm(reqSequence, reqEntryId, reqTarget, transmitter);
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
                if (rsm.needRequeue())
                {
                    ROS_WARN("Request sequence %u TIMEOUT for entry %u to %s",
                        reqSequence, reqEntryId, reqTarget.c_str());
                    reqManager->requeueReq(reqSequence, reqEntryId, reqTarget);
                }
                else if (rsm.hasReceived())
                {
                    DataMsg m = rsm.dataReceived;
                    ROS_INFO("Request sequence %u for entry %u to %s PUSH DATA",
                        reqSequence, reqEntryId, reqTarget.c_str());
                    dataAccessor->pushData(m.entryId, m.timestamp, m.data);
                }
                lifeRunning.store(false);
            }
        }

        std::lock_guard<std::mutex> lock(mRsm);
        _rsm = nullptr;
        reqManager->removeReq(reqSequence, reqEntryId, reqTarget);
    }

public:
    /**
     * @brief Construct a new Requestor object.
     * 
     * @param reqSequence Sequence number of this request.
     * @param reqEntryId Entry ID this request is made for.
     * @param reqTarget Intended target address this request is to be made to.
     * @param transmitter Interface used to send messages. 
     * @param dataAccessor Interface used to access data in database.
     * @param reqManager Interface of Requestors manager that owns this Requestor.
     */
    Requestor(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, 
        ATransmitter* transmitter, ADataAccessor* dataAccessor, AReqManager* reqManager)
        : ALifeEntity(), reqSequence(reqSequence), reqEntryId(reqEntryId), reqTarget(reqTarget), 
            transmitter(transmitter), dataAccessor(dataAccessor), reqManager(reqManager), _rsm(nullptr) 
    {
        lifeRunning.store(true);
        lifeTh = std::thread(&Requestor::_life, this);
    }

    /**
     * @brief Destroy the Requestor object.
     * 
     */
    ~Requestor()
    {
        lifeRunning.store(false);
        if (lifeTh.joinable())
        {
            lifeTh.join();
        }
    }

    /**
     * @brief Requestor is not CopyConstructible.
     * 
     */
    Requestor(const Requestor& other) = delete;

    /**
     * @brief Requestor is not CopyAssignable.
     * 
     * @param other 
     * @return Requestor& 
     */
    Requestor& operator=(const Requestor& other) = delete;

    /**
     * @brief Requestor is not MoveConstructible.
     * 
     */
    Requestor(const Requestor&& other) = delete;

    /**
     * @brief Requestor is not MoveAssignable.
     * 
     */
    Requestor& operator=(const Requestor&& other) = delete;

    /**
     * @brief Notifies the Requestor of a received ACK message.
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

    /**
     * @brief Notifies the Requestor of a received DATA message.
     * 
     * @param dataMsg DATA message received.
     * @param src Source address of this message.
     */
    void recvData(DataMsg& dataMsg, std::string src)
    {
        std::lock_guard<std::mutex> lock(mRsm);
        if (_rsm)
        {
            _rsm->recvData(dataMsg, src);
        }
    }
};

#endif // H_REQUESTOR