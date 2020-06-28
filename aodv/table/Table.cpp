#include "Table.hpp"
#include <stdint.h>

namespace aodv
{

    /**
     * @brief Construct a new object.
     * 
     */
    Route(uint8_t destAddr, uint32_t destSeq, bool isInvalid, uint8_t hopCount, uint8_t nextHop, uint8_t precursors[], uint64_t lifetime);

    Route::Route() :
        destAddr(), destSeq(), isInvalid(), hopCount(), nextHop(), precursors[](), lifetime()
    {
    }

    Route::Route(uint8_t destAddr, uint32_t destSeq, bool isInvalid, uint8_t hopCount, uint8_t nextHop, uint8_t precursors[], uint64_t lifetime) :
        destAddr(destAddr), destSeq(destSeq), isInvalid(isInvalid), hopCount(hopCount), nextHop(nextHop), precursors[](precursors), lifetime(lifetime)
    {
    }

    bool Route::check(uint8_t destAddr);
    {
        return this->destAddr == destAddr;
    }

    Table::Table();

    void Table::rcreate(Route r)
    {
        this->table[this->size++] = r;
    }

    Route Table::rread(uint8_t destAddr)
    {
        for (int i=0; i<this->size; i++) {
            Route r = this->table[i];
            if (r.destAddr == destAddr) {
                return r;
            }
        }
        return Route();
    }

    Route Table::rupdate(uint16_t i, Route r)
    {
        this->table[i] = r;
        this->size = i;
    }

    void Table::rdelete(uint16_t i)
    {
        if (i <= this->size) {
            this->table[i] = Route();
        }
    }

    void Table::clear()
    {
    } // TODO check freshness
}
