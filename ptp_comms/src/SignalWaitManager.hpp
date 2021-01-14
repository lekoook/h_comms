#ifndef H_SIGNAL_WAIT_MANAGER
#define H_SIGNAL_WAIT_MANAGER

#include <condition_variable>
#include <unordered_map>
#include <map>
#include <mutex>

namespace ptp_comms
{

/**
 * @brief Represents the identifier of a wait entry.
 * 
 */
class WaitID
{
public:
    uint32_t sequence = 0;
    uint32_t segment = 0;
    WaitID(uint32_t sequence, uint32_t segment) : sequence(sequence), segment(segment) {}

    bool operator==(const WaitID& other) const
    {
        return (sequence == other.sequence && segment == other.segment);
    }
};

/**
 * @brief Hash function for WaitID.
 * 
 */
struct WaitIDHasher
{
    std::size_t operator()(const WaitID& obj) const
    {
        using std::hash;
        return ((hash<uint32_t>()(obj.sequence) ^ (hash<uint32_t>()(obj.segment) << 1)) >> 1);
    }
};

/**
 * @brief Interface for a Waiter.
 * 
 */
class AWaiter
{
public:
    virtual void notify() = 0;
};

/**
 * @brief A class to help manage the waiting of a signal. Waiters can post a wait entry and signallers can notify that
 * waiter via that entry.
 * 
 */
class SignalWaitManager
{
private:
    /**
     * @brief A record of all waiting entry.
     * 
     */
    std::unordered_map<WaitID, AWaiter*, WaitIDHasher> record;

    /**
     * @brief Mutex to protect the record.
     * 
     */
    std::mutex mRecord;

public:
    /**
     * @brief Construct a new Signal Wait Manager object.
     * 
     */
    SignalWaitManager() {}

    /**
     * @brief Post a new waiting entry.
     * 
     * @param id Identifier for that entry.
     * @param waiter Waiter interface.
     */
    void post(WaitID id, AWaiter* waiter)
    {
        if (waiter != nullptr)
        {
            std::lock_guard<std::mutex> lock(mRecord);
            record[id] = waiter;
        }
    }

    /**
     * @brief Notifies a waiting entry.
     * 
     * @param id Identifier of entry to notify.
     */
    void signal(WaitID id)
    {
        std::lock_guard<std::mutex> lock(mRecord);
        if (record.find(id) != record.end())
        {
            record[id]->notify();
            record.erase(id);
        }
    }
};

}

#endif // H_SIGNAL_WAIT_MANAGER