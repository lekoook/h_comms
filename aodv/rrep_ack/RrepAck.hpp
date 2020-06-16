#ifndef RREP_ACK_H_
#define RREP_ACK_H_

#include <stdint.h>

namespace aodv_msgs
{
    /**
     * @brief Byte length of a RREP_ACK message.
     * 
     */
    const uint8_t RREP_ACK_LEN = 2;

    /**
     * @brief Represents the Route Reply Acknowledgement RREP_ACK Message format.
     * 
     */
    class RrepAck
    {
    private:
        /**
         * @brief RREP_ACK message type number. RREP_ACK always has '4'.
         * 
         */
        uint8_t type = 4;

    public:
        /**
         * @brief Construct a new RrepAck::RrepAck object.
         * 
         */
        RrepAck();

        /**
         * @brief Serializes a rreq:rreq object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes. MUST be of RREP_ACK_LEN length.
         */
        void serialise(uint8_t data[RREP_ACK_LEN]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the rrep::rrep object.
         * 
         * @param data uint8_t array containing the data to deserialize. MUST be of RREP_ACK_LEN length.
         */
        void deserialise(uint8_t data[RREP_ACK_LEN]);
    };
}

#endif // RREP_ACK_H