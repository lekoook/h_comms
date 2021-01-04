#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <sqlite3.h>
#include "MIT.hpp"

typedef ROW_ID_DATA std::pair<uint16_t, std::string>
typedef ROWS_ID_DATA std::vector<ROW_ID_DATA>

struct Schema {
    uint16_t id;
    uint64_t timestamp;
    std::string data;
};

class Db {
    private:
        sqlite3* db;

        /** Die if result code is an error code.
         * Do not call `sqlite3_free(messageError);`.
         * @param rc Result code.
         * @param rcNonError Result code to compare with `rc`. Default value is `SQLITE_OK`. Other values are `SQLITE_ROW`, and `SQLITE_DONE`.
         */
        void dieOnError(int32_t rc, int32_t rcNonError = SQLITE_OK) const {
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
            execute("create table if not exists metadata (id int2 primary key not null, timestamp int8 not null, data text not null);");
        }

        /** Execute only one SQL statement.
         * @param zSql SQL statement, UTF-8 encoded.
         */
        std::vector<Schema> execute(std::string zSql) const {
            std::vector<Schema> rows;
            sqlite3_stmt* stmt;
            int32_t rc;

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
        MIT execute(std::string zSql) const {
            MIT mit = MIT();
            sqlite3_stmt* stmt;
            int32_t rc;

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
        ROWS_ID_DATA execute(std::string zSql) const {
            ROWS_ID_DATA rows = ROWS_ID_DATA();
            sqlite3_stmt* stmt;
            int32_t rc;

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
        void insert(std::vector<Schema> rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "insert into metadata values(" << row.id << "," << row.timestamp << ",'" << row.data << "');";
                execute(oss.str());
            }
        }

        /** Update a vector of rows. */
        void update(std::vector<Schema> rows) {
            for (Schema row : rows) {
                std::ostringstream oss;
                oss << "update metadata set timestamp=" << row.timestamp << ", data='" << row.data << "' where id=" << row.id << ";";
                execute(oss.str());
            }
        }

        /** Select rows by their ids. */
        std::vector<Schema> select(std::vector<uint16_t> ids) {
            std::ostringstream oss;
            oss << "select * from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str());
        }

        /** Select timestamps by their ids. */
        MIT select(std::vector<uint16_t> ids) {
            std::ostringstream oss;
            oss << "select id, timestamp from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str());
        }

        /** Select data by their ids. */
        ROWS_ID_DATA select(std::vector<uint16_t> ids) {
            std::ostringstream oss;
            oss << "select id, data from metadata where id in (";
            for (size_t i=0; i<ids.size(); ++i) {
                if (i != 0) {
                    oss << ",";
                }
                oss << ids[i];
            }
            oss << ");";
            return execute(oss.str());
        }

        void print(std::vector<Schema> rows, std::ostream& os = std::cout) const {
            for (Schema row : rows) {
                os << row.id << ',' << row.timestamp << ',' << row.data << std::endl;
            }
        }

        /** Return all rows as an `ostream`. */
        friend std::ostream& operator<<(std::ostream& os, const Db& obj);
};

std::ostream& operator<<(std::ostream& os, const Db& obj) { 
    obj.print(obj.execute("select * from metadata"), os);
    return os;
}
