#include "Rerr.hpp"
#include "../utilities/serialisers.hpp"

namespace aodv_msgs
{
    Rerr::Rerr() :
    flag(), destCount(), destAddr(), destSeq()
    {
    }

    Rerr::Rerr(uint8_t flag, uint32_t destAddr, uint32_t destSeq) :
    flag(flag), destCount(), destAddr(destAddr), destSeq(destSeq)
    {
    }

    void Rerr::serialise(uint8_t data[RERR_LEN]) {
        //rerr type
        serialisers::copyU8(&data[0], type);

        //set delete flag
        uint8_t fTmp = flag << 7;
        serialisers::copyU8(&data[1], fTmp);

        //set reserved bits to 0
        serialisers::copyU8(&data[2], 0);

        //set destcount
        serialisers::copyU8(&data[3], destCount);

        //set addr of unreachable node
        serialisers::copyU32(&data[4], destAddr);

        //set seq num of unreachable node
        serialisers::copyU32(&data[8], destSeq);
    }

    void Rerr::deserialise(uint8_t data[RERR_LEN]) {
        //Get rerr type
        type = serialisers::getU8(&data[0]);

        //get flags
        uint8_t fTmp = 0;
        fTmp = serialisers::getU8(&data[1]);
        flag = fTmp >> 7;

        //get destCount
        destCount = serialisers::getU8(&data[3]);

        //get destAddr
        destAddr = serialisers::getU32(&data[4]);

        //get destSeq
        destSeq = serialisers::getU32(&data[8]);
    }

}