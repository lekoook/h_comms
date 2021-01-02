// Compile with `g++ sqlite.cpp -l sqlite3`.
#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <sqlite3.h>

struct Schema {
	uint16_t id;
	uint64_t timestamp;
	std::string data;
};

class Database {
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

	public:

		Database() {
			open();
			createTable();
		}

		~Database() {
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

		void print(std::vector<Schema> rows, std::ostream& os = std::cout) const {
			for (Schema row : rows) {
				os << row.id << ',' << row.timestamp << ',' << row.data << std::endl;
			}
		}

		/** Return all rows as an `ostream`. */
		friend std::ostream& operator<<(std::ostream& os, const Database& obj);
};

std::ostream& operator<<(std::ostream& os, const Database& obj) { 
	obj.print(obj.execute("select * from metadata"), os);
	return os;
}

int main(int argc, char** argv)
{
	Database db = Database();
	std::cout << db << std::endl;
	db.insert({Schema{1001, 1609590167, "data1"}, Schema{1002, 1609590168, "data2"}, Schema{1003, 1609590169, "data3"}, Schema{1004, 1609590170, "data4"}, Schema{1005, 1609590171, "data5"}});
	//db.update({Schema{1001, 1609590172, "data6"}, Schema{1002, 1609590173, "data7"}, Schema{1003, 1609590174, "data8"}, Schema{1004, 1609590175, "data9"}, Schema{1005, 1609590176, "data10"}});
	std::cout << db << std::endl;
}
