#ifndef H_ADATA_ACCESSOR
#define H_ADATA_ACCESSOR

#include <cstdint>
#include <vector>

/**
 * @brief Interface to access data in the database.
 * 
 */
class ADataAccessor
{
public:
    /**
     * @brief Pushes new data into the database.
     * 
     * @param entryId EntryID of this data.
     * @param timestamp Timestamp of this data.
     * @param data Actual data to push.
     * @return true If the push was successul.
     * @return false If the push was unsuccessful.
     */
    virtual bool pushData(uint16_t entryId, uint64_t timestamp, const std::vector<uint8_t>& data) = 0;

    /**
     * @brief Pulls a data from the database.
     * 
     * @param entryId EntryID of data to pull.
     * @param timestamp Reference to store timestamp of pulled data.
     * @param data Reference to store pulled data.
     * @return true If the pull was successul.
     * @return false If the pull was unsuccessful.
     */
    virtual bool pullData(uint16_t entryId, uint64_t& timestamp, std::vector<uint8_t>& data) = 0;
};

#endif // H_ADATA_ACCESSOR