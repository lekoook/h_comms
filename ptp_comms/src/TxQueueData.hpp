#ifndef H_TX_QUEUE_DATA
#define H_TX_QUEUE_DATA

#include <vector>
#include <string>
#include <cstdint>

/**
 * @brief Represents an item in the queue for data to be transmitted.
 * 
 */
class TxQueueData
{
public:
    /**
     * @brief Indicates the number of times this piece of data has tried to be transmitted.
     * 
     */
    int8_t tries = 0;

    /**
     * @brief Sequence number for this piece of data.
     * 
     */
    uint32_t seqNum = 0;

    /**
     * @brief Actual payload data to transmit.
     * 
     */
    std::vector<uint8_t> data;

    /**
     * @brief Destination of this piece of data.
     * 
     */
    std::string dest;

    /**
     * @brief Port number this piece of data should go to.
     * 
     */
    uint16_t port;

    /**
     * @brief Construct a new Tx Queue Data object.
     * 
     */
    TxQueueData () {}

    /**
     * @brief Construct a new Tx Queue Data object.
     * 
     * @param data Payload data.
     * @param dest Destination to send to.
     * @param port Port to send to.
     * @param seqNum Sequence number for this data.
     */
    TxQueueData (std::vector<uint8_t> data, std::string dest, uint16_t port, uint32_t seqNum) 
        : data(data), dest(dest), port(port), seqNum(seqNum)
    {}
};

#endif // H_TX_QUEUE_DATA