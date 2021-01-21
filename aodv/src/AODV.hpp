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
#include "ARouteObservation.hpp"
#include "WaitTimer.hpp"

namespace aodv
{

/**
 * @brief Encapsulates all specifications and workings of a Ad Hoc On-Demand Distance Vector (AODV) routing protocol.
 * 
 */
class AODV : public ARouteObserver
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
     * @brief Timer used to wait in ROS time.
     * 
     */
    WaitTimer waitTimer;

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

    /**
     * @brief Attempts to discover a route to destination node by broadcasting RREQ message(s).
     * 
     * @param destination Address of the destination node to discover route for.
     * @param waitTime The amount of time in milliseconds to wait for the RREP after broadcasting RREQ.
     * @return true If a route has been found.
     * @return false If a route cannot be found.
     */
    bool _discoverRouteOnce(std::string destination, int waitTime)
    {
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

            // We want to be notified when the route we are interested in has become valid.
            routeTable.subRouteValid(destination, this);

            // Broadcast RREQ.
            ptpClient->sendTo(ptp_comms::BROADCAST_ADDR, rreq.serialize());
        }

        // Wait for RREP to arrive.
        waitTimer.setTime((double)(waitTime / 1000.0f));
        waitTimer.wait();

        {
            std::lock_guard<std::mutex> tLock(mRouteTable);
            // We no longer care if this route has become valid.
            routeTable.unsubRouteValid(destination, this);
        }

        return waitTimer.isInterrupted();
    }

    void _rxCb(std::string src, uint16_t port, std::vector<uint8_t> data)
    {
        BaseMsg bm;
        bm.deserialize(data);
        switch (bm.msgType)
        {
        case MsgType::RReq:
            _handleRreq(src, data);
            break;
        case MsgType::RRep:
            _handleRrep(data);
            break;
        case MsgType::RRer:
            _handleRrer(data);
            break;
        case MsgType::RRepAck:
            _handleRrepAck(data);
            break;
        default:
            ROS_ERROR("Unknown AODV message type.");
        }
    }

    void _handleRreq(std::string sender, std::vector<uint8_t>& data)
    {
        RreqMsg msg;
        msg.deserialize(data);
        std::string originator = msg.getSrcAddr();
        std::string destination = msg.getDestAddr();
        ROS_INFO("Received RREQ from %s", sender.c_str());

        {
            std::lock_guard<std::mutex> tLock(mRouteTable);
            routeTable.print();
            // Create or update an entry to previous hop first.
            if (routeTable.entryExists(sender))
            {
                routeTable[sender].isValidRoute = false;
            }
            else
            {
                routeTable.upsertEntry(RouteTableEntry(sender, sender, 1, 0, 0, {}, false));
            }

            if (_hasSeenRreq(msg.rreqId, originator))
            {
                return; // We have seen this RREQ recently, discard it.
            }

            msg.hopCount++;

            // Update the route to Originator accordingly.
            if (routeTable.entryExists(originator))
            {
                auto& currEntry = routeTable[originator];
                currEntry.destSequence = std::max(msg.srcSeq, currEntry.destSequence);
                currEntry.isValidRoute = true;
                currEntry.nextHop = sender;
                currEntry.hopCount = msg.hopCount;
                uint32_t currTime = ros::Time::now().toNSec() / 1000000;
                currEntry.lifetime = std::max(
                    currEntry.lifetime, 
                    currTime + (2 * config::NET_TRAVERSAL_TIME) - (2 * msg.hopCount * config::NODE_TRAVERSAL_TIME));
            }
            else
            {
                uint32_t currTime = ros::Time::now().toNSec() / 1000000;
                uint32_t lifetime = 
                    currTime + (2 * config::NET_TRAVERSAL_TIME) - (2 * msg.hopCount * config::NODE_TRAVERSAL_TIME);
                routeTable.upsertEntry(RouteTableEntry(originator, sender, msg.hopCount, msg.srcSeq, lifetime, {}));
            }

            bool thisIsDest = destination == nodeAddr;
            bool toRrep = thisIsDest || (routeTable.isValidRoute(destination));
            if (toRrep)
            {
                if (thisIsDest)
                {
                    // Send RREP to Originator.
                    // TODO: Make the checking and generating of sequence atomic operation.
                    auto ownSeq = _currSequence();
                    if (msg.destSeq - ownSeq == 1)
                    {
                        ownSeq = _genSequence();
                    }
                    RrepMsg rrep(destination, ownSeq, originator, 0, config::MY_ROUTE_TIMEOUT, 0);
                    ptpClient->sendTo(routeTable[originator].nextHop, rrep.serialize());
                    // ROS_INFO("[DEST] Replied with RREP for %s to next hop: %s", originator.c_str(), routeTable[originator].nextHop.c_str());
                }
                else
                {
                    // Send RREP to Originator.
                    auto& destEntry = routeTable[destination];
                    auto& origEntry = routeTable[originator];
                    origEntry.precursors.push_back(destEntry.nextHop);
                    destEntry.precursors.push_back(sender);
                    uint32_t currTime = ros::Time::now().toNSec() / 1000000;
                    uint32_t lifetime = destEntry.lifetime - currTime;
                    RrepMsg rrep(destination, destEntry.destSequence, originator, destEntry.hopCount, lifetime, 0);
                    ptpClient->sendTo(origEntry.nextHop, rrep.serialize());
                    // ROS_INFO("[INTM] Replied with RREP for %s to next hop: %s", originator.c_str(), origEntry.nextHop.c_str());
                    
                    // If the RREQ has gratuitous flag set, we need to send RREP to Destination.
                    if (msg.isGratuitous)
                    {
                        RrepMsg grat(originator, msg.srcSeq, destination, origEntry.hopCount, origEntry.lifetime, 0);
                        ptpClient->sendTo(destEntry.nextHop, grat.serialize());
                        // ROS_INFO("[GRAT] Replied with RREP for %s to next hop: %s", destination.c_str(), destEntry.nextHop.c_str());
                    }
                }
            }
            else
            {
                // Update the destination sequence in the to-be-rebroadcasted RREQ.
                if (routeTable.entryExists(destination))
                {
                    auto& currEntry = routeTable[destination];
                    msg.destSeq = std::max(msg.destSeq, currEntry.destSequence);
                }
                // Rebroadcast RREQ.
                ptpClient->sendTo(ptp_comms::BROADCAST_ADDR, msg.serialize());
                // ROS_INFO("Rebroadcast RREQ for %s.", originator.c_str());
            }
        }
    }

    void _handleRrep(std::vector<uint8_t>& data)
    {
        RrepMsg msg;
        msg.deserialize(data);
    }

    void _handleRrer(std::vector<uint8_t>& data)
    {
        RrerMsg msg;
        msg.deserialize(data);
    }

    void _handleRrepAck(std::vector<uint8_t>& data)
    {
        RrepAckMsg msg;
        msg.deserialize(data);
    }

public:
    /**
     * @brief Construct a new AODV object.
     * 
     * @param nodeAddr Local address of this node.
     */
    AODV(std::string nodeAddr) : nodeAddr(nodeAddr), 
        ptpClient(std::unique_ptr<ptp_comms::PtpClient>(new ptp_comms::PtpClient(AODV_PORT, true)))
    {
        ptpClient->bind(
            std::bind(&AODV::_rxCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    void notifyRouteValid()
    {
        waitTimer.interrupt();
    }

    /**
     * @brief Begins to conduct discovery of a route to intended destination. Blocking.
     * @details This call is blocking until a route has been discovered or determined to have failed.
     * 
     * @param destination Destination address to discover route to.
     * @return true If a route was discovered.
     * @return false If no route can be found.
     */
    bool discoverRoute(std::string destination)
    {
        if (destination == nodeAddr)
        {
            return true; // Don't bother discovering our own self.
        }

        int tries = 0;
        while (tries <= config::RREQ_RETRIES)
        {
            bool res = _discoverRouteOnce(destination, config::NET_TRAVERSAL_TIME);
            if (res)
            {
                return true;
            }
            else
            {
                ros::Rate((double)config::RREQ_RATELIMIT).sleep();
            }
        }
        return false;
    }
};

}

#endif // H_AODV_AODV