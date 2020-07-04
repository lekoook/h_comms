#include "Table.hpp"
#include <stdint.h>

namespace aodv
{
    Route::Route() :
        ts(std::time(0)), destAddr(), destSeq(), isInvalid(), hopCount(), nextHop(), precursors(), lifetime()
    {
    }

    Route::Route(uint8_t destAddr, uint32_t destSeq, bool isInvalid, uint8_t hopCount, uint8_t nextHop, uint8_t precursors[256], uint64_t lifetime) :
        ts(std::time(0)), destAddr(destAddr), destSeq(destSeq), isInvalid(isInvalid), hopCount(hopCount), nextHop(nextHop), lifetime(lifetime)
    {
        // TODO copy precursor
    }

    bool Route::check(uint8_t destAddr)
    {
        return this->destAddr == destAddr;
    }

    Table::Table() :
        table(), size()
    {
    }

    void Table::rcreate(Route r)
    {
        this->table[this->size++] = r;
    }

    int Table::rsearch(uint8_t destAddr)
    {
        for (int i=0; i<this->size; i++) {
            if (this->table[i].destAddr == destAddr) {
                return i;
            }
        }
        return -1;
    }

    Route Table::rread(uint16_t i)
    {
        if (i < this->size) {
            return this->table[i];
        }
        return Route();
    }

    bool Table::rupdate(uint16_t i, Route r)
    {
        if (i < this->size) {
            this->table[i] = r;
            return true;
        }
        return false;
    }

    bool Table::rdelete(uint16_t i)
    {
        if (i < this->size) {
            this->table[i] = Route();
            return true;
        }
        return false;
    }

    void Table::clear()
    {
      /* RFC3561: section 6.4 */
      /* TODO
      std::time_t ts;
      for (int i=0; i<this->size; i++) {
          ts = this->table[i].ts;
          if (waiting_for_rrep) {
              if (ts >= std::time(0) + 2 * NET_TRAVERSAL_TIME) {
                  // TODO expunge
              }
          } else {
              if (ts >= std::time(0) + DELETE_PERIOD) {
                  // TODO expunge
              }
          }
      }
      */
    } // TODO check freshness
}
