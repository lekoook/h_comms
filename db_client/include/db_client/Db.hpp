#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <sqlite3.h>
#include "MIT.hpp"

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
    };

    // Plurals.
    typedef std::vector<uint16_t> IDS;
    typedef std::vector<ROW_ID_DATA> ROWS_ID_DATA;
    typedef std::vector<Schema> SCHEMAS;

    private:
        sqlite3* db;

        /** Die if result code is an error code.
         * Do not call `sqlite3_free(messageError);`.
         * @param rc Result code.
         * @param rcNonError Result code to compare with `rc`. Default value is `SQLITE_OK`. Other values are `SQLITE_ROW`, and `SQLITE_DONE`.
         */
        void dieOnError(RC rc, RC rcNonError = SQLITE_OK) const {
            if (rc != rcNonError) {
                std::cerr << "Dying, error: " << sqlite3_errmsg(db) << std::endl;
                exit(-1);
            }
        }

        /** Open the database. */
        void open() {
            dieOnError(sqlite3_open("db", &db));
        }

        /** Close the database. */
        void close() {
            dieOnError(sqlite3_close(db));
        }

        /** Create table if it does not exist. */
        void createTable() {
            executeSchemas("create table if not exists metadata (id int2 primary key not null, timestamp int8 not null, data text not null);");
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         */
        SCHEMAS executeSchemas(std::string zSql) const {
            SCHEMAS rows;
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            dieOnError(rc);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                rows.push_back(Schema{
                        sqlite3_column_int(stmt, 0),
                        sqlite3_column_int(stmt, 1),
                        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2))
                        });
            }

            dieOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return rows;
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         * @return MIT.
         */
        MIT executeMit(std::string zSql){
            MIT mit;
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            dieOnError(rc);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                mit.update(sqlite3_column_int(stmt, 0), sqlite3_column_int(stmt, 1));
            }

            dieOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return mit;
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         * @return ROWS_ID_DATA.
         */
        ROWS_ID_DATA executeData(std::string zSql) const {
            ROWS_ID_DATA rows = ROWS_ID_DATA();
            sqlite3_stmt* stmt;
            RC rc;

            rc = sqlite3_prepare_v2(db, zSql.c_str(), -1, &stmt, NULL);
            dieOnError(rc);
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                rows.push_back(ROW_ID_DATA{
                        sqlite3_column_int(stmt, 0),
                        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2))
                        });
            }

            dieOnError(rc, SQLITE_DONE);
            sqlite3_finalize(stmt);
            return rows;
        }

    public:

        Db() {
            open();
            createTable();
        }

        ~Db() {
            close();
        }

        /** Insert a vector of rows. */
        void insert(SCHEMAS rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "insert into metadata values(" << row.id << "," << row.timestamp << ",'" << row.data << "');";
                executeSchemas(oss.str());
            }
        }

        /** Update a vector of rows. */
        void update(SCHEMAS rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "update metadata set timestamp=" << row.timestamp << ", data='" << row.data << "' where id=" << row.id << ";";
                executeSchemas(oss.str());
            }
        }

        /** Select rows by their ids. */
        SCHEMAS selectSchemas(IDS ids) {
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
        MIT selectMit(IDS ids) {
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
        ROWS_ID_DATA selectData(IDS ids) {
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
        MIT selectMit() {
            return executeMit("select id, timestamp from metadata;");
        }

        /** Select all data. */
        ROWS_ID_DATA selectData() {
            return executeData("select id, data from metadata;");
        }
};
