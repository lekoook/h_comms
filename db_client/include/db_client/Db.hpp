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
class Db {
    public:
    // Singles.
    typedef int32_t RC;
    typedef uint16_t ID;
    typedef uint64_t TIMESTAMP;
    typedef std::string DATA;
    typedef std::pair<ID, DATA> ROW_ID_DATA;
    struct Schema {
        ID id;
        TIMESTAMP timestamp;
        DATA data;

        Schema() {}
        Schema(ID id, TIMESTAMP timestamp, DATA data) : id(id), timestamp(timestamp), data(data) {}
    };

    // Plurals.
    typedef std::vector<uint16_t> IDS;
    typedef std::vector<ROW_ID_DATA> ROWS_ID_DATA;
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
                        sqlite3_column_int(stmt, 1),
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
                mit.update(sqlite3_column_int(stmt, 0), sqlite3_column_int(stmt, 1));
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

        Db(std::string robotName) {
            open(robotName);
            createTable();
        }

        ~Db() {
            close();
        }

        /** Upsert a vector of rows.
         * Requires SQLite version >=3.24.0.
         */
        tl::optional<bool> upsert(SCHEMAS rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "insert into metadata values(" << row.id << "," << row.timestamp << ",'" << row.data << "') on conflict(id) do update set timestamp=excluded.timestamp, data=excluded.data;";
                if (!executeSchemas(oss.str())) {
                    return tl::nullopt;
                }
            }
            return false;
        }

        /** Select rows by their ids. */
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

        /** Select timestamps by their ids. */
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

        /** Select data by their ids. */
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

        /** Select all timestamps. */
        tl::optional<MIT> selectMit() {
            return executeMit("select id, timestamp from metadata;");
        }

        /** Select all data. */
        tl::optional<ROWS_ID_DATA> selectData() {
            return executeData("select id, data from metadata;");
        }
};
}
