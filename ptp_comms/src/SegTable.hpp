#include <map>
#include <vector>
#include <utility>
#include <mutex>
#include "Packet.hpp"

namespace ptp_comms
{

/**
 * @brief Represents a table that tracks and reconstructs multiple segments uniquely identified by a address and sequence
 * number.
 * 
 */
class SegTable
{
private:
    /**
     * @brief This std::map represents the table used to identify existing stored segments for each unique address and 
     * sequence number pair.
     * @details The key is a unique address and sequence number pair. The value is a bytes std::vector, bitset and 
     * timestamp (seconds) tuple. 
     * The bytes std::vector will contain the actual data and the bitset is used to determine which segment has
     * arrived and stored. The index of bitset corresponds to the segment number and a set bit indicates that the 
     * segment has arrived and vice versa. The timestamp indicates the time at which this entry was last updated. It is 
     * used to determine if this entry is too old and subjected for deletion.
     * 
     */
    std::map<std::pair<std::string, uint32_t>, std::tuple<std::vector<uint8_t>, std::vector<bool>, uint32_t>> table;

    /**

     * @brief Mutex for table.
     * 
     */
    std::mutex mTable;
    
public:
    /**
     * @brief Construct a new Seg Table object.
     * 
     */
    SegTable() {}

    /**
     * @brief Updates the segment table with a new segment.
     * 
     * @param addr Address that this segment belongs to.
     * @param segment Segment of the Packet type.
     * @param timestamp Timestamp at which this update occurs.
     * @return true If this segment was the last segment and the full data has been reconstructed.
     * @return false If the full has not yet been fully reconstructed.
     */
    bool updateSeg(std::string addr, const Packet& segment, uint32_t timestamp)
    {
        std::pair<std::string, uint32_t> key = std::make_pair(addr, segment.seqNum);

        std::lock_guard<std::mutex> lock(mTable);
        if (table.find(key) != table.end())
        {
            auto& value = table[key];
            std::copy(segment.data.begin(), segment.data.end(), std::get<0>(value).begin() + segment.segNum * Packet::MAX_SEGMENT_SIZE);
            std::get<1>(value)[segment.segNum] = true;
        }
        else
        {
            // No exisiting record, make a new entry with enough reserved space for the whole data.
            std::vector<uint8_t> whole(segment.totalLength, 0);
            // Insert this first arrived segment.
            std::copy(segment.data.begin(), segment.data.end(), whole.begin() + segment.segNum * Packet::MAX_SEGMENT_SIZE);
            // Construct the bitset.
            std::vector<bool> bitset(
                (segment.totalLength / Packet::MAX_SEGMENT_SIZE) + ((segment.totalLength % Packet::MAX_SEGMENT_SIZE) != 0), 
                false);
            bitset[segment.segNum] = true;

            table[key] = std::make_tuple(whole, bitset, timestamp);
        }

        // Check if all bits in bitset has been set.
        for (bool b : std::get<1>(table[key]))
        {
            if (!b)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Attempts to get the full data of an entry identified by the address and sequence number.
     * @details Caller must check that the full data has indeed been reconstructed otherwise this will return the current state of the full data.
     * 
     * @param addr Address of entry.
     * @param sequence Sequence number of entry.
     * @return std::vector<uint8_t> Full data as a bytes vector.
     */
    std::vector<uint8_t> getFullData(std::string addr, uint32_t sequence)
    {
        std::lock_guard<std::mutex> lock(mTable);
        auto key = std::make_pair(addr, sequence);
        std::vector<uint8_t> ret = std::get<0>(table[key]);
        table.erase(key);
        return ret;
    }

    /**
     * @brief Checks if an entry has its data fully reconstructed.
     * 
     * @param addr Address of entry.
     * @param sequence Sequence number of entry.
     * @return true If the entry has its data fully reconstructed.
     * @return false If the entry has its data not fully reconstructed.
     */
    bool isFullData(std::string addr, uint32_t sequence)
    {
        std::pair<std::string, uint32_t> key = std::make_pair(addr, sequence);
        std::lock_guard<std::mutex> lock(mTable);
        // Check if all bits in bitset has been set.
        for (bool b : std::get<1>(table[key]))
        {
            if (!b)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Iterates through and erases entries that are too old.
     * 
     * @param maxAge Maximum age (seconds) that will be considered old.
     * @param currentTime Current time (seconds) at which this operation is performed.
     */
    void clean(uint32_t maxAge, uint32_t currentTime)
    {
        std::lock_guard<std::mutex> lock(mTable);
        for (auto it = table.begin(); it != table.end();)
        {
            uint32_t et = std::get<2>(it->second);
            if ((currentTime - et > maxAge))
            {
                it = table.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
};

}
