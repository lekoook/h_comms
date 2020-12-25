#ifndef H_LIFE_ENTITY
#define H_LIFE_ENTITY

#include <thread>
#include <atomic>
#include "ATransmitter.hpp"

class ALifeEntity
{
protected:
    /**
     * @brief Life thread that runs for the whole duration of the life entity until it destructs.
     * 
     */
    std::thread lifeTh;

    /**
     * @brief Flag to indicate if the life thread is running.
     * 
     */
    std::atomic<bool> lifeRunning;

    /**
     * @brief Executes the lifetime of the life entity.
     * 
     */
    virtual void _life() = 0;

public:
    /**
     * @brief Destroy the ALifeEntity object.
     * 
     */
    virtual ~ALifeEntity()
    {
        lifeRunning.store(false);
        if (lifeTh.joinable())
        {
            lifeTh.join();
        }
    }


    /**
     * @brief Check if this life entity has ended it's entire sequence.
     * 
     * @return true If the life of the entity has ended.
     * @return false If the life of the entity has not yet ended.
     */
    bool hasEnded()
    {
        return !lifeRunning.load();
    }
};

#endif // H_LIFE_ENTITY