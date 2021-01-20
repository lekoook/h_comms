#ifndef H_AODV_AODV
#define H_AODV_AODV

#include "ros/ros.h"
#include <cstdint>
#include <mutex>
#include <memory>
#include <map>
#include "ptp_comms/PtpClient.hpp"
#include "ConfigParams.hpp"
#include "RouteTable.hpp"
#include "messages/RreqMsg.hpp"
#include "messages/RrepMsg.hpp"
#include "messages/RrerMsg.hpp"
#include "messages/RrepAckMsg.hpp"

namespace aodv
{

/**
 * @brief Encapsulates all specifications and workings of a Ad Hoc On-Demand Distance Vector (AODV) routing protocol.
 * 
 */
class AODV
{
private:
    /**
     * @brief Port number used by AODV application.
     * 
     */
    uint16_t const AODV_PORT = 654;

    /**
     * @brief Local address of this node.
     * 
     */
    std::string nodeAddr = "";

    /**
     * @brief The unique identifier number for each RREQ message originating from this node.
     * 
     */
    uint32_t rreqId = 0;

    /**
     * @brief Mutex to protect this node's RREQ identifier.
     * 
     */
    std::mutex mRreqId;

    /**
     * @brief Sequence number for this AODV node.
     * 
     */
    uint32_t sequence = 0;

    /**
     * @brief Mutex to protect this node's sequence number.
     * 
     */
    std::mutex mSequence;

    /**
     * @brief Route table to track all possible routes.
     * 
     */
    RouteTable routeTable;

    /**
     * @brief Mutex to protect the route table.
     * 
     */
    std::mutex mRouteTable;

    /**
     * @brief Buffer to track all RREQ that was seen previously. Used to prevent re-processing of the same RREQ.
     * 
     */
    std::map<std::pair<uint32_t, std::string>, uint32_t> seenRreq;

    /**
     * @brief Mutex to protect seen RREQ buffer.
     * 
     */
    std::mutex mSeenRreq;

    /**
     * @brief Communication client used to transmit and receive messages.
     * 
     */
    std::unique_ptr<ptp_comms::PtpClient> ptpClient = nullptr;

    /**
     * @brief Internal method to generate the next available sequence number for this node.
     * 
     * @return uint32_t New Sequence number.
     */
    uint32_t _genSequence()
    {
        std::lock_guard<std::mutex> lock(mSequence);
        return ++sequence;
    }

    /**
     * @brief Internal method to check the current sequence number for this node.
     * 
     * @return uint32_t Current sequence number.
     */
    uint32_t _currSequence()
    {
        std::lock_guard<std::mutex> lock(mSequence);
        return sequence;
    }

    /**
     * @brief Internal method to generate the next available RREQ ID for this node.
     * 
     * @return uint32_t New RREQ ID.
     */
    uint32_t _genRreqId()
    {
        std::lock_guard<std::mutex> lock(mRreqId);
        return ++rreqId;
    }

    /**
     * @brief Internal method to check the current RREQ ID for this node.
     * 
     * @return uint32_t Current RREQ ID.
     */
    uint32_t _currRreqId()
    {
        std::lock_guard<std::mutex> lock(mRreqId);
        return rreqId;
    }

    /**
     * @brief Buffers the RREQ ID and the Originator address to prevent the same RREQ from being re-processed.
     * 
     * @param rreqId RREQ ID in RREQ message.
     * @param originator Originator address in RREQ message.
     */
    void _bufferRreq(uint32_t rreqId, std::string originator)
    {
        std::lock_guard<std::mutex> lock(mSeenRreq);
        seenRreq[std::make_pair(rreqId, originator)] = ros::Time::now().toNSec() / 1000000;
    }

    /**
     * @brief Queries if the RREQ ID and the Originator address was seen previously and processed.
     * 
     * @param rreqId RREQ ID in RREQ message.
     * @param originator Originator address in RREQ message.
     * @return true If the RREQ was seen previously and processed.
     * @return false If the RREQ was never seen before.
     */
    bool _hasSeenRreq(uint32_t rreqId, std::string originator)
    {
        std::lock_guard<std::mutex> lock(mSeenRreq);
        // First remove all entries that are too old.
        uint32_t currentTime = ros::Time::now().toNSec() / 1000000;
        for (auto it = seenRreq.begin(); it != seenRreq.end();)
        {
            if (currentTime - it->second > (uint32_t)config::PATH_DISCOVERY_TIME)
            {
                it = seenRreq.erase(it);
            }
            else
            {
                it++;
            }
        }
        // Now we check.
        return seenRreq.find(std::make_pair(rreqId, originator)) != seenRreq.end();
    }

public:
    /**
     * @brief Construct a new AODV object.
     * 
     * @param nodeAddr Local address of this node.
     */
    AODV(std::string nodeAddr) : nodeAddr(nodeAddr), 
        ptpClient(std::unique_ptr<ptp_comms::PtpClient>(new ptp_comms::PtpClient(AODV_PORT, true))) {}

    /**
     * @brief Attempts to discover a route to destination node by broadcasting RREQ message(s).
     * 
     * @param destination Address of the destination node to discover route for.
     * @param waitTime The amount of time in milliseconds to wait for the RREP after broadcasting RREQ.
     * @return true If a route has been found.
     * @return false If a route cannot be found.
     */
    bool discoverRouteOnce(std::string destination, int waitTime)
    {
        std::lock_guard<std::mutex> tLock(mRouteTable);
        if (routeTable.isValidRoute(destination))
        {
            return true; // Don't discover if there already exists a route.
        }

        // Create RREQ message.
        RreqMsg rreq;
        bool hasEntry = routeTable.entryExists(destination);
        if (hasEntry)
        {
            RouteTableEntry entry;
            routeTable.getEntry(destination, &entry);
            rreq.destSeq = entry.destSequence;
        }
        rreq.setDestAddr(destination);
        rreq.isUnkSequence = !hasEntry;
        rreq.isGratuitous = true;
        rreq.setSrcAddr(nodeAddr);
        rreq.srcSeq = _genSequence();
        rreq.rreqId = _genRreqId();
        rreq.hopCount = 0;
        
        // Prevent ourselves from processing our own RREQ.
        _bufferRreq(rreq.rreqId, nodeAddr);

        // TODO: Do we need a way to identify that RREP we receive after this corresponds to the RREQ we sent?

        // Broadcast RREQ.
        ptpClient->sendTo(ptp_comms::BROADCAST_ADDR, rreq.serialize());

        // TODO: Wait for RREP to arrive.
    }
};

}

#endif // H_AODV_AODV