#ifndef MIT_HEADER_
#define MIT_HEADER_

#include <unordered_map>
#include <utility>
#include <vector>
#include <set>

/**
 * @brief Meta-Information Table (MIT) records the freshness of each piece of datum (of a particular type) generated by each robot.
 * 
 * The freshness of a datum generated by a robot is determined by the timestamp (milliseconds, 64 bits unsigned int) at which that robot generates that datum.
 * The fresher the datum, the higher the timestamp value will be.
 * 
 * The MIT has a map data structure:
 * 
 * ------------------------
 * | Entry ID | Timestamp |
 * ------------------------
 * |  10001   |   10000   |
 * ------------------------
 * |  10002   |   20000   |
 * ------------------------
 * |  30003   |   30000   |
 * ------------------------
 * 
 * Where each Entry ID (16 bits unsigned int) will map to a timestamp (64 bits unsigned int) value.
 * 
 * The Entry ID can be decomposed into Robot ID and Data Type number:
 * 
 * Robot ID - A unique identifier assigned to a robot.
 * Data Type number - Represents a particular datum type (user defines what the type corresponds to which number).
 * 
 * The first 2 digits of Entry ID holds the Robot ID and the last 3 digits holds the Data Type number.
 * For example, an Entry ID of "10002" represents Robot ID of "10" and Data Type of "002" (or just "2").
 * NOTE: Since Entry ID is 16 bits unsigned int, the maximum number of robots representable is 66 and the maximum number of data types representable is 536.
 * 
 */
class MIT
{
private: 
    /**
     * @brief Unordered map with the entry ID as the key and timestamp as the value.
     * 
     */
    std::unordered_map<uint16_t, uint64_t> table;

    /**
     * @brief Check if robot ID and data type number is in valid range. Raise std::out_of_range if it is outside valid range.
     * 
     * @param robotId Robot ID to check.
     */
    void checkIds(uint16_t robotId, uint16_t dataType)
    {
        if (robotId > 65)
        {
            throw std::out_of_range("robotId argument value must be in the range: [0:65]");
        }
        if (dataType > 535)
        {
            throw std::out_of_range("dataType argument value must be in the range: [0:535]");
        }
    }

    /**
     * @brief Check if entry ID is in valid. Raise std::out_of_range if either robot ID or data type number components are outside valid range.
     * 
     * @param entryId Entry ID to check.
     */
    void checkEntryId(uint16_t entryId)
    {
        uint16_t robotId;
        uint16_t dataType;
        convertFromEntryId(entryId, robotId, dataType);
        checkIds(robotId, dataType);
    }

public:
    /**
     * @brief Construct a new MIT object.
     * 
     */
    MIT()
    {
    }

    /**
     * @brief Converts robot ID and data type number to entry ID.
     * 
     * @param robotId Robot ID to convert with.
     * @return uint16_t Converted entry ID.
     */
    uint16_t convertToEntryId(uint16_t robotId, uint16_t dataType)
    {
        return robotId * 1000 + dataType;
    }

    /**
     * @brief Converts robot ID and data type number to entry ID.
     * 
     * @param entryId Entry ID to convert from.
     * @param robotId Reference to Robot ID to convert to.
     */
    uint16_t convertFromEntryId(uint16_t entryId, uint16_t& robotId, uint16_t& dataType)
    {
        robotId = entryId / 1000;
        dataType = entryId % 1000;
    }

    /**
     * @brief Updates a robot ID and data type number pair along with the timestamp information into the table. Create a new entry if it does not to exist.
     * 
     * @param robotId Robot ID to insert.
     * @param timestamp Timestamp information to insert.
     */
    void update(uint16_t robotId, uint16_t dataType, uint64_t timestamp)
    {
        checkIds(robotId, dataType);
        uint16_t entryId = convertToEntryId(robotId, dataType);
        table[entryId] = timestamp;
    }

    /**
     * @brief Get a set of table entry IDs.
     * 
     * @return std::set<uint16_t> Set of entry IDs current in the table.
     */
    std::set<uint16_t> getIdKeys()
    {
        std::set<uint16_t> ret;
        for (auto& item: table)
        {
            ret.insert(item.first);
        }
        return ret;
    }

    /**
     * @brief Gets the timestamp of an entry.
     * 
     * @param robotId Robot ID of entry.
     * @param dataType Data type number of entry.
     * @return uint64_t Timestamp for that entry.
     */
    uint64_t getTimestamp(uint16_t robotId, uint16_t dataType)
    {
        return table[convertToEntryId(robotId, dataType)];
    }

    /**
     * @brief Gets the timestamp of an entry.
     * 
     * @param entryId Entry ID of entry.
     * @return uint64_t Timestamp for that entry.
     */
    uint64_t getTimestamp(uint16_t entryId)
    {
        return table[entryId];
    }

    /**
     * @brief Compares this MIT with the MIT in the argument. If argument MIT has an entry that we don't or the timestamp is higher, mark the entry.
     * 
     * @param toCompare MIT to compare with.
     * @return std::vector<std::pair<uint16_t, uint16_t>> A vector of robot ID (first) and data type number (second) pairs.
     */
    std::vector<std::pair<uint16_t, uint16_t>> compare(MIT toCompare)
    {
        std::set<uint16_t> compareKeys = toCompare.getIdKeys();
        std::vector<std::pair<uint16_t, uint16_t>> ret;

        // TODO: Lock guard this->table, so during comparison, no external changes can be made until comparison with another MIT is done.

        for (auto& key : compareKeys)
        {
            if (table.count(key) == 0 || table[key] < toCompare.getTimestamp(key))
            {
                uint16_t robId;
                uint16_t datType;
                convertFromEntryId(key, robId, datType);
                ret.push_back(std::make_pair(robId, datType));
            }
        }

        return ret;
    }

    /**
     * @brief Serialises the MIT into a byte (8 bits unsigned int) vector. Order of entries is not guaranteed.
     * 
     * @return std::vector<uint8_t> Serialised byte vector.
     */
    std::vector<uint8_t> serialise()
    {
        std::vector<uint8_t> ret;

        for (auto& item: table)
        {
            ret.push_back((uint8_t)(item.first & 0xFF));
            ret.push_back((uint8_t)(item.first >> 8 & 0xFF));
            ret.push_back((uint8_t)(item.second & 0xFF));
            ret.push_back((uint8_t)(item.second >> 8 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 16 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 24 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 32 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 40 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 48 & 0xFF));
            ret.push_back((uint8_t)(item.second >> 56 & 0xFF));
        }

        return ret;
    }

    /**
     * @brief Deserialises a byte (8 bits unsigned int) vector into the MIT. Order of entries after deserialising is not guaranteed.
     * @details This will replace the entire inner record table. Trailing bytes that cannot form an entry will be ignored.
     * 
     * @param buffer Byte vector to deserialise.
     */
    void deserialise(std::vector<uint8_t> buffer)
    {
        table.clear();
        int siz = buffer.size();
        int i = 0;

        while (siz - i >= 10)
        {
            uint16_t robotId = buffer[i] | (buffer[i+1] << 8);
            uint64_t timestamp = buffer[i+2] 
                | ((uint64_t)buffer[i+3] << 8) 
                | ((uint64_t)buffer[i+4] << 16)
                | ((uint64_t)buffer[i+5] << 24)
                | ((uint64_t)buffer[i+6] << 32)
                | ((uint64_t)buffer[i+7] << 40)
                | ((uint64_t)buffer[i+8] << 48)
                | ((uint64_t)buffer[i+9] << 56);
            table[robotId] = timestamp;
        }
    }
};

#endif // MIT_HEADER_