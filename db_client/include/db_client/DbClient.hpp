#ifndef H_DB
#define H_DB

#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <sqlite3.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "MIT.hpp"
#include "tl/optional.hpp"

/** Return tl::nullopt if result code is an error code.
 * Do not call `sqlite3_free(messageError);`.
 * @param rc Result code.
 * @param rcNonError Result code to compare with `rc`. Default value is `SQLITE_OK`. Other values are `SQLITE_ROW`, and `SQLITE_DONE`.
 */
#define returnNulloptOnError(rc, rcNonError) do { \
    if (rc != rcNonError) { \
        std::cerr << "Error: " << sqlite3_errmsg(db) << std::endl; \
        return tl::nullopt; /* return optional object that does not contain a value. */ \
    } \
} while (0)

namespace db_client {

/**
 * @brief  Database for all shared data.
 * 
 */
class DbClient {
    public:
    /**
     * @brief Result code.
     * 
     */
    typedef int32_t RC;

    /**
     * @brief ID of an entry.
     * 
     */
    typedef uint16_t ID;

    /**
     * @brief Timestamp of an entry.
     * 
     */
    typedef uint64_t TIMESTAMP;

    /**
     * @brief Data of an entry.
     * 
     */
    typedef std::string DATA;

    /**
     * @brief Entry ID and data pair.
     * 
     */
    typedef std::pair<ID, DATA> ROW_ID_DATA;

    /**
     * @brief Structure to represent the parameters of one single entry.
     * 
     */
    struct Schema {
        ID id;
        TIMESTAMP timestamp;
        DATA data;

        Schema() {}
        Schema(ID id, TIMESTAMP timestamp, DATA data) : id(id), timestamp(timestamp), data(data) {}
    };

    /**
     * @brief Multiple entry IDs.
     * 
     */
    typedef std::vector<uint16_t> IDS;

    /**
     * @brief Multiple entry ID and data pairs.
     * 
     */
    typedef std::vector<ROW_ID_DATA> ROWS_ID_DATA;

    /**
     * @brief Multiple Schemas.
     * 
     */
    typedef std::vector<Schema> SCHEMAS;

    private:
        sqlite3* db;

        /** Open the database. */
        tl::optional<bool> open(std::string robotName) {
            std::string filename = "";

            // https://stackoverflow.com/a/26696759
            const char *homedir;
            if ((homedir = getenv("HOME")) == NULL) {
                homedir = getpwuid(getuid())->pw_dir;
            }

            filename.append(homedir);
            filename.append("/" + robotName + ".sqlite3");

            returnNulloptOnError(sqlite3_open(filename.c_str(), &db), SQLITE_OK); // return optional object that does not contain a value.
            return false; // return optional object that contains false.
        }

        /** Close the database. */
        tl::optional<bool> close() {
            returnNulloptOnError(sqlite3_close(db), SQLITE_OK);
            return false;
        }

        /** Create table if it does not exist. */
        tl::optional<bool> createTable() {
            if (!executeSchemas("create table if not exists metadata (id int2 primary key not null, timestamp int8 not null, data text not null);")) { // If call returns nullopt, ...
                return tl::nullopt; // ... then function returns nullopt.
            }
            return false;
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         */
        tl::optional<SCHEMAS> executeSchemas(std::string zSql) {
            SCHEMAS rows;
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            returnNulloptOnError(rc, SQLITE_OK);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                rows.push_back(Schema{
                        sqlite3_column_int(stmt, 0),
                        sqlite3_column_int64(stmt, 1),
                        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2))
                        });
            }

            returnNulloptOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return rows;
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         * @return MIT.
         */
        tl::optional<MIT> executeMit(std::string zSql){
            MIT mit;
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            returnNulloptOnError(rc, SQLITE_OK);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                mit.update(sqlite3_column_int(stmt, 0), sqlite3_column_int64(stmt, 1));
            }

            returnNulloptOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return mit;
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         * @return ROWS_ID_DATA.
         */
        tl::optional<ROWS_ID_DATA> executeData(std::string zSql) {
            ROWS_ID_DATA rows = ROWS_ID_DATA();
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            returnNulloptOnError(rc, SQLITE_OK);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                rows.push_back(ROW_ID_DATA{
                        sqlite3_column_int(stmt, 0),
                        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2))
                        });
            }

            returnNulloptOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return rows;
        }

    public:
        /**
         * @brief Construct a new Db object.
         * 
         * @param name Name for this database.
         */
        DbClient(std::string name) {
            open(name);
            createTable();
        }

        /**
         * @brief Destroy the Db object.
         * 
         */
        ~DbClient() {
            close();
        }

        /**
         * @brief Inserts multiple new entries. If entry ID already exists, updates the timestamp and data.
         * 
         * @param entries Schemas to insert or update.
         * @return true If insert or update successful.
         * @return false If insert or update unsuccessful.
         */
        bool upsert(SCHEMAS entries) {
            for (Schema row : entries) {
                if (!upsert(row.id, row.timestamp, row.data)) {
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Inserts a new entry. If entry ID already exists, updates the timestamp and data.
         * 
         * @param entryId Entry ID.
         * @param timestamp Timestamp of entry.
         * @param data Data of entry.
         * @return true If insert or update successful.
         * @return false If insert or update unsuccessful.
         */
        bool upsert(ID entryId, TIMESTAMP timestamp, DATA data) {
            std::ostringstream oss;
            oss << "insert into metadata values(" << entryId << "," << timestamp << ",'" << data << "') on conflict(id) do update set timestamp=excluded.timestamp, data=excluded.data;";
            if (!executeSchemas(oss.str())) {
                return false;
            }
            return true;
        }

        /**
         * @brief Inserts a new entry. If entry ID already exists, updates the timestamp and data.
         * 
         * @param entry Entry parameters in Schema struct.
         * @return true If insert or update successful.
         * @return false If insert or update unsuccessful.
         */
        bool upsert(Schema entry) {
            return upsert(entry.id, entry.timestamp, entry.data);
        }

        /**
         * @brief Selects one entry as Schema.
         * 
         * @param entryId Entry ID to select.
         * @return tl::optional<Schema> Schema selected.
         */
        tl::optional<Schema> selectSchema(ID entryId) {
            std::ostringstream oss;
            oss << "select * from metadata where id in (" << entryId << ");";
            auto res = executeSchemas(oss.str());
            if (res && res.value().size() > 0) {
                return res.value()[0];
            }
            return tl::nullopt;
        }

        /**
         * @brief Selects multiple entries as Schemas.
         * 
         * @param ids Entry IDs to select.
         * @return tl::optional<SCHEMAS> Multiple Schemas selected.
         */
        tl::optional<SCHEMAS> selectSchemas(IDS ids) {
            std::ostringstream oss;
            oss << "select * from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return executeSchemas(oss.str());
        }

        /**
         * @brief Gets the MIT specified by the entry IDs.
         * 
         * @param ids Entry IDs to include in the MIT.
         * @return tl::optional<MIT> MIT of specified entries.
         */
        tl::optional<MIT> selectMit(IDS ids) {
            std::ostringstream oss;
            oss << "select id, timestamp from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return executeMit(oss.str());
        }

        /**
         * @brief Selects a data specified by the entry ID.
         * 
         * @param entryId Entry ID of data to select.
         * @return tl::optional<ROW_ID_DATA> Entry ID and data selected.
         */
        tl::optional<ROW_ID_DATA> selectData(ID entryId) {
            std::ostringstream oss;
            oss << "select id, data from metadata where id in (" << entryId << ");";
            auto res = executeData(oss.str());
            if (res && res.value().size() > 0) {
                return res.value()[0];
            }
            return tl::nullopt;
        }

        /**
         * @brief Selects multiple data specified by their entry IDs.
         * 
         * @param ids Entry IDs of data to select.
         * @return tl::optional<ROWS_ID_DATA> Entry IDs and data selected.
         */
        tl::optional<ROWS_ID_DATA> selectData(IDS ids) {
            std::ostringstream oss;
            oss << "select id, data from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return executeData(oss.str());
        }

        /**
         * @brief Gets the MIT based on the current database state.
         * 
         * @return tl::optional<MIT> MIT of current database state.
         */
        tl::optional<MIT> selectMit() {
            return executeMit("select id, timestamp from metadata;");
        }

        /**
         * @brief Selects all data in the database.
         * 
         * @return tl::optional<ROWS_ID_DATA> All data selected.
         */
        tl::optional<ROWS_ID_DATA> selectData() {
            return executeData("select id, data from metadata;");
        }
};
}

#endif // H_DB