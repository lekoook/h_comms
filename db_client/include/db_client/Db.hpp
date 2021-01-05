#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <sqlite3.h>
#include "MIT.hpp"

class Db {

    // Singles.
    typedef RC int32_t;
    typedef ID uint16_t;
    typedef TIMESTAMP uint64_t;
    typedef DATA std::string;
    typedef ROW_ID_DATA std::pair<ID, DATA>;
    struct Schema {
        ID id;
        TIMESTAMP timestamp;
        DATA data;
    };

    // Plurals.
    typedef IDS std::vector<uint16_t>;
    typedef ROWS_ID_DATA std::vector<ROW_ID_DATA>;
    typedef SCHEMAS std::vector<Schema>;

    // Define tags per https://en.cppreference.com/w/cpp/thread/lock_tag
    struct SUBROW_SCHEMA_T { explicit SUBROW_SCHEMA_T() = default; };
    struct SUBROW_TIMESTAMP_T { explicit SUBROW_TIMESTAMP_T() = default; };
    struct SUBROW_DATA_T { explicit SUBROW_DATA_T() = default; };
    inline constexpr SUBROW_SCHEMA_T SUBROW_SCHEMA {};
    inline constexpr SUBROW_TIMESTAMP_T SUBROW_TIMESTAMP {};
    inline constexpr SUBROW_DATA_T SUBROW_DATA {};

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
            execute("create table if not exists metadata (id int2 primary key not null, timestamp int8 not null, data text not null);", SUBROW_SCHEMA);
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         */
        SCHEMAS execute(std::string zSql, SUBROW_SCHEMA_T t) const {
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
        MIT execute(std::string zSql, SUBROW_MIT_T t) const {
            MIT mit = MIT();
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
        ROWS_ID_DATA execute(std::string zSql, SUBROW_DATA_T t) const {
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
                execute(oss.str(), SUBROW_SCHEMA);
            }
        }

        /** Update a vector of rows. */
        void update(SCHEMAS rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "update metadata set timestamp=" << row.timestamp << ", data='" << row.data << "' where id=" << row.id << ";";
                execute(oss.str(), SUBROW_SCHEMA);
            }
        }

        /** Select rows by their ids. */
        SCHEMAS select(IDS ids, SUBROW_SCHEMA_T t) {
            std::ostringstream oss;
            oss << "select * from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str(), t);
        }

        /** Select timestamps by their ids. */
        MIT select(IDS ids, SUBROW_MIT_T t) {
            std::ostringstream oss;
            oss << "select id, timestamp from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str(), t);
        }

        /** Select data by their ids. */
        ROWS_ID_DATA select(IDS ids, SUBROW_DATA_T t) {
            std::ostringstream oss;
            oss << "select id, data from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str(), t);
        }

        /** Select all timestamps. */
        MIT select(SUBROW_MIT_T t) {
            return execute("select id, timestamp from metadata;", t);
        }

        /** Select all data. */
        ROWS_ID_DATA select(SUBROW_DATA_T t) {
            return execute("select id, data from metadata;", t);
        }

        void print(SCHEMAS rows, std::ostream& os = std::cout) const {
            for (Schema row : rows) {
                os << row.id << ',' << row.timestamp << ',' << row.data << std::endl;
            }
        }

        /** Return all rows as an `ostream`. */
        friend std::ostream& operator<<(std::ostream& os, const Db& obj);
};

std::ostream& operator<<(std::ostream& os, const Db& obj) { 
    obj.print(obj.execute("select * from metadata", SUBROW_SCHEMA), os);
    return os;
}
