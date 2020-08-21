#ifndef ACKMSG_H_
#define ACKMSG_H_

#include <stdint.h>

namespace aodv
{
    class AckMsg
    {
        private:
        /**
         * @brief Specifies the type of message this is. Ack messages are always '2'. Used as a field in the message.
         * 
         */
        static const uint8_t Type = 2;  

        /**
         * @brief Index position of the \ref Type field in the message bytes array.
         * 
         */
        static const uint8_t TypeIdx = 0;
        
        /**
         * @brief Index position of the \ref seq field in the message bytes array.
         * 
         */
        static const uint8_t SeqIdx = 1;

        /**
         * @brief Index position of the \ref segSeq field in the message bytes array.
         * 
         */
        static const uint8_t SegSeqIdx = 5;


        public:
        /**
         * @brief Sequence number field of the message to acknowledge.
         * 
         */
        uint32_t seq;

        /**
         * @brief Segment sequence number field of the message to acknowledge.
         * 
         */
        uint32_t segSeq;

        /**
         * @brief Specifies the size of an Ack message. The size should contain the collective size of all fields, in bytes.
         * 
         */
        static const uint8_t AckMsgSize = 9;

        /**
         * @brief Construct a new Ack Msg object using the sequence and segment sequence number.
         * 
         * @param sequence Sequence number.
         * @param segmentSequence Segment sequence number.
         */
        AckMsg(uint32_t sequence, uint32_t segmentSequence);

        /**
         * @brief Construct a new Ack Msg object using the serialised Ack message. Effectively deserialises the Ack message.
         * 
         * @param ackMsg Serialised Ack message to deserialise.
         */
        AckMsg(uint8_t ackMsg[AckMsgSize]);

        /**
         * @brief Serialises this Ack message into a bytes array.
         * 
         * @param ackMsg Bytes array to contain the serialised message. Must have at least the size of \p AckMsgSize.
         */
        void serialise(uint8_t ackMsg[AckMsgSize]);

        /**
         * @brief Checks that every field are the same.
         * 
         * @param ackMsg The other AckMsg object to check against.
         * @return true If all fields are the same.
         * @return false If at least one field is different.
         */
        bool operator==(const AckMsg& ackMsg);
    };
}

#endif // ACKMSG_H_