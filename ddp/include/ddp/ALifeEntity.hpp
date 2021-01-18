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
     * @brief Flag to indicate if this entity needs to be relived.
     * 
     */
    std::atomic<bool> lifeRelive;

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

    /**
     * @brief Check if this life entity needs to restart it's entire sequence.
     * 
     * @return true If the entity needs to be restarted.
     * @return false If the entity does not need to be restarted.
     */
    bool needRelive()
    {
        return lifeRelive.load();
    }

    /**
     * @brief Starts the sequence of the life entity.
     * 
     */
    virtual void start() = 0;
};

#endif // H_LIFE_ENTITY