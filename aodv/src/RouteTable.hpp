#ifndef H_AODV_ROUTE_TABLE
#define H_AODV_ROUTE_TABLE

#include <unordered_map>
#include <string>
#include <tuple>
#include <cstdint>
#include <mutex>
#include <list>

namespace aodv
{

/**
 * @brief Represents an entry in AODV routing table.
 * @details The \p destination field will be the unique identifier for the entry. No two or more entries can have the 
 * same \p destination.
 * 
 */
class RouteTableEntry
{
public:
    /**
     * @brief Destination address of the entry. This uniquely identifies the entry.
     * 
     */
    std::string destination = "";

    /**
     * @brief The next address to send to in order to reach the destination address.
     * 
     */
    std::string nextHop = "";

    /**
     * @brief The number of hops for this route.
     * 
     */
    uint8_t hopCount = 0;

    /**
     * @brief The sequence number of the destination node.
     * 
     */
    uint32_t destSequence = 0;

    /**
     * @brief The lifetime of this entry.
     * 
     */
    uint32_t lifetime = 0;

    /**
     * @brief List of neighbouring nodes that would want to reach the destination address via this node.
     * 
     */
    std::list<std::string> precursors;

    /**
     * @brief Flag that indicates if this destination sequence and hence route is valid.
     * 
     */
    bool isValidRoute = true;

    /**
     * @brief Construct a new Route Table Entry object.
     * 
     */
    RouteTableEntry() {}
    
    /**
     * @brief Construct a new Route Table Entry object.
     * 
     * @param destination Destination address of the entry.
     * @param nextHop The next address to send to in order to reach the destination address.
     * @param hopCount The number of hops for this route.
     * @param destSequence The sequence number of the destination node.
     * @param lifetime The lifetime of this entry.
     * @param precursors List of neighbouring nodes that would want to reach the destination address via this node.
     * @param isValidRoute Flag that indicates if this route is valid.
     */
    RouteTableEntry(std::string destination, std::string nextHop, uint8_t hopCount, 
        uint32_t destSequence, uint32_t lifetime, std::list<std::string> precursors, bool isValidDestSequence=true)
        : destination(destination), nextHop(nextHop), hopCount(hopCount), destSequence(destSequence), 
        lifetime(lifetime), precursors(precursors), isValidRoute(isValidRoute)
    {}
};

/**
 * @brief Represents the AODV routing table.
 * 
 */
class RouteTable
{
private:
    /**
     * @brief Internal class representation of the table entry.
     * 
     */
    class Entry
    {
    public:
        std::string nextHop = "";
        uint8_t hopCount = 0;
        uint32_t destSequence = 0;
        uint32_t lifetime = 0;
        std::list<std::string> precursors;
        bool isValidRoute = true;

        Entry() {}

        Entry(std::string nextHop, uint8_t hopCount, uint32_t destSequence, uint32_t lifetime, 
            std::list<std::string> precursors, bool isValidRoute) 
            : nextHop(nextHop), hopCount(hopCount), destSequence(destSequence), lifetime(lifetime), 
            precursors(precursors), isValidRoute(isValidRoute)
        {}
    };

    /**
     * @brief Routing table itself. The key is the destination address, the value is the rest of the entry fields for 
     * the routing table.
     * 
     */
    std::unordered_map<std::string, Entry> table;
    
    /**
     * @brief Internal method to insert or replace (if already exists) an entry.
     * 
     * @param destination Destination (unique key) of entry.
     * @param nextHop Next hop address.
     * @param hopCount Hop count.
     * @param destSequence Destination sequence.
     * @param lifetime Lifetime of entry.
     * @param precursors Precursors (immediate neighbours) that would want to reach that destination via this node.
     * @param isValidRoute Flag that indicates if the destination sequence is valid.
     */
    void _insertEntry(std::string destination, std::string nextHop, uint8_t hopCount, uint32_t destSequence, 
        uint32_t lifetime, std::list<std::string> precursors, bool isValidRoute)
    {
        table[destination] = Entry(nextHop, hopCount, destSequence, lifetime, precursors, isValidRoute);
    }

    /**
     * @brief Internal method to query if if a particular entry exists.
     * 
     * @param destination Destination (unique key) of the entry to query.
     * @return true If such entry exists.
     * @return false If such entry don't exists.
     */
    bool _exists(std::string destination)
    {
        return table.find(destination) != table.end();
    }

public:
    /**
     * @brief Construct a new Route Table object.
     * 
     */
    RouteTable() {}

    /**
     * @brief Queries if a particular entry exists.
     * 
     * @param destination Destination (unique key) of the entry to query.
     * @return true If such entry exists.
     * @return false If such entry don't exists.
     */
    bool entryExists(std::string destination)
    {
        return _exists(destination);
    }

    /**
     * @brief Inserts or updates (if already exists) an entry.
     * 
     * @param entry Entry to insert or update with.
     */
    void upsertEntry(RouteTableEntry entry)
    {
        _insertEntry(entry.destination, entry.nextHop, entry.hopCount, entry.destSequence, entry.lifetime, 
            entry.precursors, entry.isValidRoute);
    }

    /**
     * @brief Gets the entry specified by the destination.
     * 
     * @param destination Destination (unique key) of the entry.
     * @param outEntry Output argument that stores the retrieved (if any) entry.
     * @return true If such entry exists.
     * @return false If such entry don't exists.
     */
    bool getEntry(std::string destination, RouteTableEntry* outEntry)
    {
        if (_exists(destination))
        {
            auto& entry = table[destination];
            outEntry->destination = destination;
            outEntry->nextHop = entry.nextHop;
            outEntry->hopCount = entry.hopCount;
            outEntry->destSequence = entry.destSequence;
            outEntry->lifetime = entry.lifetime;
            outEntry->precursors = entry.precursors;
            outEntry->isValidRoute = entry.isValidRoute;
            return true;
        }
        return false;
    }

    /**
     * @brief Queries if a route to the specified destination exists (and is valid).
     * 
     * @param destination Destination address to query.
     * @return true If a route exists and is valid.
     * @return false If no route exists or it exists but not valid.
     */
    bool isValidRoute(std::string destination)
    {
        if (_exists(destination))
        {
            auto& entry = table[destination];
            return entry.isValidRoute;
        }
        return false;
    }
};

}

#endif // H_AODV_ROUTE_TABLE