#ifndef H_AODV_ROUTE_TABLE
#define H_AODV_ROUTE_TABLE

#include <unordered_map>
#include <string>
#include <tuple>
#include <cstdint>
#include <mutex>
#include <list>
#include "ARouteObservation.hpp"

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
     * @brief Set of neighbouring nodes that would want to reach the destination address via this node.
     * 
     */
    std::set<std::string> precursors;

    /**
     * @brief Flag that indicates if this destination sequence is valid.
     * 
     */
    bool isValidDestSeq = false;

    /**
     * @brief Flag that indicates if this route is an active route.
     * 
     */
    bool isActiveRoute = false;

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
     * @param isValidDestSeq Flag that indicates if this destination sequence is valid.
     * @param isActiveRoute Flag that indicates if this route is active.
     */
    RouteTableEntry(std::string destination, std::string nextHop, uint8_t hopCount, 
        uint32_t destSequence, uint32_t lifetime, std::set<std::string> precursors, bool isValidDestSeq=false, bool isActiveRoute=false)
        : destination(destination), nextHop(nextHop), hopCount(hopCount), destSequence(destSequence), 
        lifetime(lifetime), precursors(precursors), isValidDestSeq(isValidDestSeq), isActiveRoute(isActiveRoute)
    {}
};

/**
 * @brief Represents the AODV routing table.
 * 
 */
class RouteTable : public ARouteSubject
{
public:
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
        std::set<std::string> precursors;
        bool isValidDestSeq = true;
        bool isActiveRoute = false;

        Entry() {}

        Entry(std::string nextHop, uint8_t hopCount, uint32_t destSequence, uint32_t lifetime, 
            std::set<std::string> precursors, bool isValidDestSeq, bool isActiveRoute) 
            : nextHop(nextHop), hopCount(hopCount), destSequence(destSequence), lifetime(lifetime), 
            precursors(precursors), isValidDestSeq(isValidDestSeq), isActiveRoute(isActiveRoute)
        {}
    };

private:
    /**
     * @brief Routing table itself. The key is the destination address, the value is the rest of the entry fields for 
     * the routing table.
     * 
     */
    std::unordered_map<std::string, Entry> table;

    /**
     * @brief List of observers looking for validity of each route.
     * 
     */
    std::unordered_map<std::string, std::list<ARouteObserver*>> routeValidObservers;
    
    /**
     * @brief Internal method to insert or replace (if already exists) an entry.
     * 
     * @param destination Destination (unique key) of entry.
     * @param nextHop Next hop address.
     * @param hopCount Hop count.
     * @param destSequence Destination sequence.
     * @param lifetime Lifetime of entry.
     * @param precursors Precursors (immediate neighbours) that would want to reach that destination via this node.
     * @param isValidDestSeq Flag that indicates if this destination sequence is valid.
     * @param isActiveRoute Flag that indicates if this route is active.
     */
    void _insertEntry(std::string destination, std::string nextHop, uint8_t hopCount, uint32_t destSequence, 
        uint32_t lifetime, std::set<std::string> precursors, bool isValidDestSeq, bool isActiveRoute)
    {
        bool prevInvalid = (table.find(destination) == table.end()) || !table[destination].isActiveRoute;
        table[destination] = Entry(nextHop, hopCount, destSequence, lifetime, precursors, isValidDestSeq, isActiveRoute);
        if (isActiveRoute && prevInvalid)
        {
            ROS_INFO("notifying");
            _notifyRouteActive(destination);
        }
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

    /**
     * @brief Notify all observers looking at the specified destination address.
     * 
     * @param destination Destination address to notify for.
     */
    void _notifyRouteActive(std::string destination)
    {
        if (routeValidObservers.find(destination) != routeValidObservers.end())
        {
            for (auto ob : routeValidObservers[destination])
            {
                ob->notifyRouteActive();
            }
        }
    }

public:
    /**
     * @brief Construct a new Route Table object.
     * 
     */
    RouteTable() {}

    /**
     * @brief Overloads the [] operator.
     * 
     * @param destination Destination address.
     * @return Entry& Reference to the entry.
     */
    Entry& operator[](std::string destination)
    {
        return table[destination];
    }

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
            entry.precursors, entry.isValidDestSeq, entry.isActiveRoute);
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
            outEntry->isValidDestSeq = entry.isValidDestSeq;
            outEntry->isActiveRoute = entry.isActiveRoute;
            return true;
        }
        return false;
    }

    /**
     * @brief Queries if a route to the specified destination exists (and is active).
     * 
     * @param destination Destination address to query.
     * @return true If a route exists and is valid.
     * @return false If no route exists or it exists but not valid.
     */
    bool isActiveRoute(std::string destination)
    {
        if (_exists(destination))
        {
            auto& entry = table[destination];
            return entry.isActiveRoute;
        }
        return false;
    }

    /**
     * @brief Queries if a route to the specified destination exists has valid destination sequence.
     * 
     * @param destination Destination address to query.
     * @return true If a route exists and destination sequence is valid.
     * @return false If no route exists or it exists but destination sequence is not valid.
     */
    bool isValidDestSeq(std::string destination)
    {
        if (_exists(destination))
        {
            auto& entry = table[destination];
            return entry.isValidDestSeq;
        }
        return false;
    }

    /**
     * @brief Subscribe to be notified of route changing from invalid to valid.
     * 
     * @param destination Destination address of route to be notified of.
     * @param observer Observer interface.
     */
    void subRouteValid(std::string destination, ARouteObserver* observer)
    {
        routeValidObservers[destination].push_back(observer);
    }

    /**
     * @brief Unsubscribe to be notified of route changing from invalid to valid.
     * 
     * @param destination Destination address of route to be notified of.
     * @param observer Observer interface.
     */
    void unsubRouteValid(std::string destination, ARouteObserver* observer)
    {
        if (routeValidObservers.find(destination) != routeValidObservers.end())
        {
            routeValidObservers[destination].remove(observer);
        }
    }

    void print()
    {
        std::cout << "==== Route Table ====" << std::endl;
        std::cout << "Dest\tNH\t" << std::endl;
        for (auto entry : table)
        {
            std::cout << entry.first << "\t" << entry.second.nextHop << "\t" << std::endl;
        }
        std::cout << "== Route Table End ==" << std::endl;
    }
};

}

#endif // H_AODV_ROUTE_TABLE