#ifndef H_GRAPH_MSG
#define H_GRAPH_MSG

#include <vector>
#include <cstdint>
#include "BaseMsg.hpp"
#include <iostream>
#include <rosmsg_compressor/rosmsg_serializer.h>
#include "exchange_graphs/GeometryGraphStamped.h"

/**
 * @brief Represents the graph message.
 * 
 * Has the same fields as `exchange_graphs::GeometryGraphStamped`.
 */
class GraphMsg : public BaseMsg
{
private:
    /**
     * @brief Total length of fixed-length fields.
     * @details Length = MsgType(1)
     * 
     */
    const uint8_t FIXED_LEN = 1;

public:
    typedef exchange_graphs::GeometryGraphStamped GRAPH_STAMPED;

    /**
     * @brief The graph.
     * 
     */
    GRAPH_STAMPED learntGraphStamped;

    /**
     * @brief Construct a new Graph Msg object.
     * 
     */
    GraphMsg() : BaseMsg(MsgType::Graph) {}

    /**
     * @brief Construct a new Graph Msg object
     * 
     * @param graph The graph.
     */
    GraphMsg(GRAPH_STAMPED learntGraphStamped) : BaseMsg(MsgType::Graph), learntGraphStamped(learntGraphStamped) {}

    /**
     * @brief Serializes the graph message into a bytes vector.
     * 
     * @return std::vector<uint8_t> Serialized bytes vector.
     */
    std::vector<uint8_t> serialize()
    {
        std::vector<uint8_t> buffer;
        RosMsgCompressor::serialize_to_byte_array(this->learntGraphStamped, buffer);
        uint32_t len = buffer.size() + this->FIXED_LEN;
        uint8_t temp[len];

        serialisers::copyU8(temp, type);
        memcpy(&temp[this->FIXED_LEN], buffer.data(), buffer.size());

        return std::vector<uint8_t>(temp, temp + len);
    }

    /**
     * @brief Deserializes a bytes vector into a graph message.
     * 
     * @param bytes Bytes vector to deserialize.
     */
    void deserialize(std::vector<uint8_t>& bytes)
    {
        uint8_t* temp = bytes.data();
        uint8_t t = serialisers::getU8(&temp[0]);
        
        if (t != MsgType::Graph)
        {
            throw std::invalid_argument("Message is not of MsgType::Graph!");
        }
        std::vector<uint8_t> bytesTail(bytes.begin() + this->FIXED_LEN, bytes.end());
        RosMsgCompressor::deserialize_from_byte_array(bytesTail, this->learntGraphStamped);
    }
};

#endif // H_GRAPH_MSG
