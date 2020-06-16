#ifndef RREP_H_
#define RREP_H_

#include <stdint.h>

namespace aodv_msgs
{
    /**
     * @brief Repair flag mask.
     * 
     */
    const uint8_t REPR_F_MASK = 0b00000010u;

    /**
     * @brief Acknowledgement flag mask.
     * 
     */
    const uint8_t ACK_F_MASK = 0b00000001u;

    /**
     * @brief Byte length of a RREP message.
     * 
     */
    const uint8_t RREP_LEN = 20;

    /**
     * @brief Represents the Route Reply RREP Message format.
     * 
     */
    class Rrep
    {
    private:
        /**
         * @brief RREP message type number. RREP always has '2'.
         * 
         */
        uint8_t type = 2;

    public:
        /**
         * @brief RREP flag bits.
         * 
         */
        uint8_t flags = 0;

        /**
         * @brief RREP Prefix Size.
         * 
         */
        uint8_t prefixSz = 0;

        /**
         * @brief RREP hop count.
         * 
         */
        uint8_t hopCount = 0;

        /**
         * @brief RREP destination address.
         * 
         */
        uint32_t destAddr = 0;

        /**
         * @brief RREP destination sequence number.
         * 
         */
        uint32_t destSeq = 0;

        /**
         * @brief RREP source address.
         * 
         */
        uint32_t srcAddr = 0;

        /**
         * @brief RREP lifetime.
         * 
         */
        uint32_t lifetime = 0;

        /**
         * @brief Construct a new rreq::rreq object with all members zero initialised.
         * 
         */
        Rrep();

        /**
         * @brief Construct a new rreq::rreq object.
         * 
         * @param flags flag bits.
         * @param prefixSz prefix size.
         * @param hopCount hop count.
         * @param destAddr destination address.
         * @param destSeq destination sequence number.
         * @param srcAddr source address.
         * @param lifetime source sequence number.
         */
        Rrep(uint8_t flags, uint8_t prefixSz, uint8_t hopCount, 
            uint32_t destAddr, uint32_t destSeq, uint32_t srcAddr, uint32_t lifetime);

        /**
         * @brief Serializes a rreq:rreq object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes. MUST be of RREP_LEN length.
         */
        void serialise(uint8_t data[RREP_LEN]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the rrep::rrep object.
         * 
         * @param data uint8_t array containing the data to deserialize. MUST be of RREP_LEN length.
         */
        void deserialise(uint8_t data[RREP_LEN]);
    };
}

#endif // RREP_H