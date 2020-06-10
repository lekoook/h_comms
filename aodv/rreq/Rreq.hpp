#ifndef RREQ_H_
#define RREQ_H_

#include <stdint.h>

namespace aodv_msgs
{
    /**
     * @brief Join flag mask.
     * 
     */
    const uint8_t JOIN_F_MASK = 0b00010000u;

    /**
     * @brief Repair flag mask.
     * 
     */
    const uint8_t REPR_F_MASK = 0b00001000u;

    /**
     * @brief Gratuitous flag mask.
     * 
     */
    const uint8_t GRAT_F_MASK = 0b00000100u;

    /**
     * @brief Destination flag mask.
     * 
     */
    const uint8_t DEST_F_MASK = 0b00000010u;

    /**
     * @brief Unknown sequence number flag mask.
     * 
     */
    const uint8_t USEQ_F_MASK = 0b00000001u;

    /**
     * @brief Byte length of a RREQ message.
     * 
     */
    const uint8_t RREQ_LEN = 24;

    class Rreq
    {
    private:
        /**
         * @brief RREQ message type number. RREQ always has '1'.
         * 
         */
        uint8_t type = 1;

        /**
         * @brief Copies a uint8_t value into a uint8_t bytes array.
         * 
         * @param dest uint8_t array to store the copied bytes. MUST be at least 1 byte length.
         * @param src uint8_t value to copy.
         */
        void copyU8(uint8_t* dest, uint8_t src);

        /**
         * @brief Copies a uint32_t value into a uint8_t bytes array.
         * 
         * @param dest uint8_t array to store the copied bytes. MUST be at least 4 bytes length.
         * @param src uint32_t value to copy.
         */
        void copyU32(uint8_t* dest, uint32_t src);

        /**
         * @brief Retrieves a uint8_t value from a uint8_t bytes array.
         * 
         * @param src uint8_t array to retrieve from. MUST be at least 1 byte length.
         * @return uint8_t value retrieved.
         */
        uint8_t getU8(uint8_t* src);

        /**
         * @brief Retrieves a uint32_t value from a uint8_t bytes array.
         * 
         * @param src uint8_t array to retrieve from. MUST be at least 4 bytes length.
         * @return uint32_t value retrieved.
         */
        uint32_t getU32(uint8_t* src);

    public:
        /**
         * @brief RREQ flag bits.
         * 
         */
        uint8_t flags = 0;
        
        /**
         * @brief RREQ hop count.
         * 
         */
        uint8_t hopCount = 0;
        
        /**
         * @brief RREQ identifier.
         * 
         */
        uint32_t id = 0;

        /**
         * @brief RREQ destination address.
         * 
         */
        uint32_t destAddr = 0;

        /**
         * @brief RREQ destination sequence number.
         * 
         */
        uint32_t destSeq = 0;

        /**
         * @brief RREQ source address.
         * 
         */
        uint32_t srcAddr = 0;

        /**
         * @brief RREQ source sequence number.
         * 
         */
        uint32_t srcSeq = 0;

        /**
         * @brief Construct a new rreq::rreq object with all members zero initialised.
         * 
         */
        Rreq();

        /**
         * @brief Construct a new rreq::rreq object.
         * 
         * @param flags flag bits.
         * @param hopCount hop count.
         * @param id identifier.
         * @param destAddr destination address.
         * @param destSeq destination sequence number.
         * @param srcAddr source address.
         * @param srcSeq source sequence number.
         */
        Rreq(uint8_t flags, uint8_t hopCount, uint32_t id, 
            uint32_t destAddr, uint32_t destSeq, uint32_t srcAddr, uint32_t srcSeq);

        /**
         * @brief Serializes a rreq:rreq object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes. MUST be of RREQ_LEN length.
         */
        void serialise(uint8_t data[RREQ_LEN]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the rreq::rreq object.
         * 
         * @param data uint8_t array containing the data to deserialize. MUST be of RREQ_LEN length.
         */
        void deserialise(uint8_t data[RREQ_LEN]);
    };
}

#endif // RREQ_H