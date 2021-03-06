#ifndef TABLE_H_
#define TABLE_H_

#include "../config.h"
#include <ctime>
#include <stdint.h>

namespace aodv
{
    class Route
    {
    private:
  
    public:
        /**
         * @brief timestamp, creation time.
         * 
         */
        std::time_t ts;

        /**
         * @brief destination address.
         * 
         */
        uint8_t destAddr;
  
        /**
         * @brief destination sequence number.
         * 
         */
        uint32_t destSeq;

        /**
         * @brief valid destination sequence number flag.
         *
         * An invalid route is used to store previously valid route information for an extended period of time.
         * An invalid route cannot be used to forward data packets, but it can provide information useful for route repairs, and also for future RREQ messages.
         */
        bool validSequenceNumber;

        /* TODO other flags */

        /**
         * @brief hop count.
         * 
         */
        uint8_t hopCount;
  
        /**
         * @brief Next hop address.
         * 
         */
        uint8_t nextHop;
  
        /**
         * @brief list of precursors.
         *
         * List of precursors that may be forwarding packets on this route.
         * These precursors will receive notifications from the node in the event of detection of the loss of the next hop link.
         * The list of precursors in a routing table entry contains those neighboring nodes to which a route reply was generated or forwarded.
         * 
         * Has length 2**8 because with a uint8_t mac address, the network diameter is at most 2**8.
         */
        uint8_t precursors[256];

        /**
         * @brief expiration of deletion time of the route.
         *
         */
        uint64_t lifetime = ACTIVE_ROUTE_TIMEOUT;

        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Route();
  
        /**
         * @brief Construct a new object.
         * 
         */
        Route(uint8_t destAddr, uint32_t destSeq, bool validSequenceNumber, uint8_t hopCount, uint8_t nextHop, uint8_t precursors[], uint64_t lifetime);

        /**
         * @brief Check expected destAddr with actual destAddr.
         *
         * @param destAddr expected destAddr.
         */
        bool check(uint8_t destAddr);
    };

    class Table
    {
    private:
        /**
         * @brief A table is an array of routes.
         *
         * Has length 2**16 because with a uint8_t mac address, there are at most 2**8 * 2**8 rows in the table.
         */
        Route table[65536];
  
        /**
         * @brief Current number of routes in the table.
         */
        uint16_t size;
  
    public:
        /**
         * @brief Construct a new object.
         * 
         */
        Table();

        /**
         * @brief Appends the route at the end of the table.
         *
         * @param r route
         */
        void rcreate(Route r);

        /**
         * @brief Linear search through table to find the index with a matching destination address.
         *
         * @param destAddr destination address
         * @return nonnegative index if found, else negative index
         */
        int rsearch(uint8_t destAddr);

        /**
         * @brief Get route.
         *
         * @param i index
         * @return route if @param i is < the table size, else empty route
         */
        Route rread(uint16_t i);

        /**
         * @brief Replace route at index @param i with new route @param r.
         *
         * Does nothing except return false if @param i is >= the table size.
         *
         * @param i index
         * @param r new route
         * @return true if i<size, else false
         */
        bool rupdate(uint16_t i, Route r);

        /**
         * @brief Replace the route at index @param i with an empty route.
         *
         * Does nothing except return false if @param i is >= the table size.
         *
         * @param i index
         * @return true if i<size, else false
         */
        bool rdelete(uint16_t i);

        /**
         * @brief Clears some routes from the table.
         *
         * An expired routing table entry SHOULD NOT be expunged before (current_time + DELETE_PERIOD) (see section 6.11).
         * Otherwise, the soft state corresponding to the route (e.g., last known hop count) will be lost.
         * Furthermore, a longer routing table entry expunge time MAY be configured.
         * Any routing table entry waiting for a RREP SHOULD NOT be expunged before (current_time + 2 * NET_TRAVERSAL_TIME).
         */
        void clear();
    };
}

#endif // TABLE_H_
