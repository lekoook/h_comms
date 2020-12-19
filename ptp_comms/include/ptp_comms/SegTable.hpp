#include <map>
#include <vector>
#include <utility>
#include <mutex>
#include "Packet.hpp"

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
     * @details The key is a unique address and sequence number pair. The value is a bytes std::vector and bitset pair. 
     * The bytes std::vector will contain the actual data and the bitset is used to determine which segment has
     * arrived and stored. The index of bitset corresponds to the segment number and a set bit indicates that the 
     * segment has arrived and vice versa.
     * 
     */
    std::map<std::pair<std::string, uint32_t>, std::pair<std::vector<uint8_t>, std::vector<bool>>> table;

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
     * @return true If this segment was the last segment and the full data has been reconstructed.
     * @return false If the full has not yet been fully reconstructed.
     */
    bool updateSeg(std::string addr, const Packet& segment)
    {
        std::pair<std::string, uint32_t> key = std::make_pair(addr, segment.seqNum);

        std::lock_guard<std::mutex> lock(mTable);
        if (table.find(key) != table.end())
        {
            auto& value = table[key];
            std::copy(segment.data.begin(), segment.data.end(), value.first.begin() + segment.segNum * Packet::MAX_SEGMENT_SIZE);
            value.second[segment.segNum] = true;
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

            table[key] = std::make_pair(whole, bitset);
        }

        // Check if all bits in bitset has been set.
        for (bool b : table[key].second)
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
        std::vector<uint8_t> ret = table[key].first;
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
        for (bool b : table[key].second)
        {
            if (!b)
            {
                return false;
            }
        }
        return true;
    }
};