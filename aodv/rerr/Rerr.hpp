#ifndef RERR_H_
#define RERR_H_
#include <stdint.h>

namespace aodv_msgs
{   

    /**
     * @brief No delete flag mask.
     * 
     */ 
    const uint8_t No_DELETE_F_MASK = 0b00000001;
    /**
     * @brief byte length of a RERR message, assuming only 1 dest.
     * 
     */ 
    const uint8_t RERR_LEN = 12;

    class Rerr
    {
    private:
        /**
         * @brief RERR message type number. RERR always has '3'.
         * 
         */
        uint8_t type = 3;

    public:
        /**
         * @brief Flag bit
         * 
         */
        uint8_t flag = 0;
        /**
         * @brief Number of unreachable destinations. Default to 1.
         * 
         */
        uint8_t destCount = 1;

        /**
         * @brief Address of unreachable node.
         * 
         */ 
        uint32_t destAddr = 0;

        /**
         * @brief sequence number for unreachable destination.
         * 
         */ 
        uint32_t destSeq = 0;

        /**
         * @brief Construct a new rerr:rerr object with all params set to 0.
         * 
         */ 
        Rerr();

        /**
         * @brief Construct a new rerr:rerr object.
         * 
         * @param destAddr destination address.
         * @param destSeq destination sequence number.
         */
        Rerr(uint8_t flag, uint32_t destAddr, uint32_t destSeq);

        /**
         * @brief Serializes a rreq:rreq object into a uint8_t bytes array.
         * 
         * @param data uint8_t array to store the bytes. MUST be of RREQ_LEN length.
         */
        void serialise(uint8_t data[RERR_LEN]);

        /**
         * @brief Deserializes the given uint8_t bytes array into the rreq::rreq object.
         * 
         * @param data uint8_t array containing the data to deserialize. MUST be of RREQ_LEN length.
         */
        void deserialise(uint8_t data[RERR_LEN]);
    };
}

#endif // RERR_H