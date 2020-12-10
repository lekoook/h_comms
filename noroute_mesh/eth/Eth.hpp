#ifndef ETH_H_
#define ETH_H_

#include <stdint.h>
#include <string>
#include <vector>

namespace aodv
{
    /**
     * @brief Byte length of the non-variable part of a Eth.
     * 
     */
    const uint8_t ETH_NONVAR_LEN = 23;

    class Eth
    {
    private:
        /**
         * @brief crc
         *
         */
        uint32_t crc;
  
    public:
        /**
         * @brief sequence number
         * 
         */
        uint32_t seq;

        /**
         * @brief segment sequence number
         * 
         */
        uint32_t segSeq;

        /**
         * @brief exact maximum segment sequence number.
         *
         * There are segSeqMax segments in one packet.
         * Design choice:
         *   segSeqMax instead of boolean flag indicating end of all segments in a packet.
         *   Because segments can arrive out of order, so the amount of memory needed to hold all segments is unknown until the last segment arrives.
         *   If segSeqMax is in all segments, then the amount of memory needed to hold all segments is known whenever any segment arrives.
         */
        uint32_t segSeqMax;

        /**
         * @brief Flag to indicate if this message is an acknowledge message.
         * @details The \p seq and \p segSeq accompanied with this acknowledge message indicates which message this is meant to acknowledge.
         * 
         */
        bool isAck;
  
        /**
         * @brief dst length
         * 
         */
        uint16_t dstLength;
  
        /**
         * @brief destination address
         * 
         */
        std::string dst;
  
        /**
         * @brief src length
         * 
         */
        uint16_t srcLength;
  
        /**
         * @brief source address
         * 
         */
        std::string src;
  
        /**
         * @brief payload length
         * 
         */
        uint16_t payloadLength;

        /**
         * @brief payload
         *
         * Must be a string.
         * If were uint8_t[], would give double free or corruption error.
         */
        std::string payload;

        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Eth();

        /**
         * @brief Construct a new non-ACK Ethernet frame.
         * 
         * @param seq Sequence number of frame.
         * @param segSeq Segment sequence number of frame.
         * @param segSeqMax Maximum number of segments.
         * @param dstLength 
         * @param dst 
         * @param srcLength 
         * @param src 
         * @param payloadLength 
         * @param isAck Indicates if this is acknowledgem message. If true, \p segSeqMax and \p payloadLength will default to zero.
         */
        Eth(uint32_t seq, uint32_t segSeq, uint32_t segSeqMax, uint16_t dstLength, std::string dst, uint16_t srcLength, std::string src, uint16_t payloadLength, bool isAck=false);

        /**
         * @brief Serializes a object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes.
         */
        void serialise(uint8_t data[]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the object.
         *
         * @param data uint8_t array containing the data to deserialize.
         */
        void deserialise(uint8_t data[]);

        /**
         * @brief Check expected crc with actual crc.
         *
         * @param crc expected crc.
         */
        bool check(uint32_t crc);

        bool operator==(const aodv::Eth& eth);
    };
}

#endif // ETH_H_
