#ifndef H_REQUESTOR
#define H_REQUESTOR

#include <thread>
#include "ALifeEntity.hpp"
#include "ReqMachine.hpp"
#include "ReqStates.hpp"
#include "ADataAccessor.hpp"

class Requestor : public ALifeEntity
{
private:
    /**
     * @brief Maximum number of times this Requestor can execute it's sequence.
     * 
     */
    int const MAX_LIVE_COUNT = 3;

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
     * @brief Tracks the number of times this Requestor has started it's sequence.
     * 
     */
    int liveCount = 0;

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
        liveCount++;
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
                    if (liveCount < MAX_LIVE_COUNT)
                    {
                        lifeRelive = true;
                    }
                }
                else if (rsm.hasReceived())
                {
                    DataMsg m = rsm.dataReceived;
                    ROS_INFO("Request sequence %u for entry %u to %s PUSH DATA",
                        reqSequence, reqEntryId, reqTarget.c_str());
                    dataAccessor->pushData(m.entryId, m.timestamp, m.data);
                }
                lifeRunning = false;
            }
        }

        std::lock_guard<std::mutex> lock(mRsm);
        _rsm = nullptr;
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
     */
    Requestor(uint32_t reqSequence, uint16_t reqEntryId, std::string reqTarget, 
        ATransmitter* transmitter, ADataAccessor* dataAccessor)
        : ALifeEntity(), reqSequence(reqSequence), reqEntryId(reqEntryId), reqTarget(reqTarget), 
            transmitter(transmitter), dataAccessor(dataAccessor), _rsm(nullptr)
    {
        lifeRunning = false;
        start();
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

    /**
     * @brief Begins the execution of this Requestor's sequence.
     * 
     */
    virtual void start()
    {
        if (lifeRunning.load())
        {
            return;
        }

        if (lifeTh.joinable())
        {
            lifeTh.join();
        }
        
        lifeRelive = false;
        lifeRunning = true;
        lifeTh = std::thread(&Requestor::_life, this);
    }

    /**
     * @brief Returns the sequence number associated with this Requestor.
     * 
     * @return uint32_t Sequence number.
     */
    uint32_t getSequence()
    {
        return reqSequence;
    }

    /**
     * @brief Returns the entry ID associated with this Requestor.
     * 
     * @return uint16_t Entry ID.
     */
    uint16_t getEntryId()
    {
        return reqEntryId;
    }

    /**
     * @brief Returns the target address associated with this Requestor.
     * 
     * @return std::string Target address.
     */
    std::string getTarget()
    {
        return reqTarget;
    }
};

#endif // H_REQUESTOR