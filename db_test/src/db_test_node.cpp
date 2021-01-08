#include "ros/ros.h"
#include <iostream>
#include <db_client/DbClient.hpp>
#include <vector>
#include <cstdint>
#include <thread>
#include <chrono>

std::string ns;
std::unique_ptr<db_client::DbClient> db;
uint16_t robotId;

void slp()
{
    using namespace std::chrono;
    int64_t slp = rand() % 2001 + 1000;
    std::this_thread::sleep_for(milliseconds(slp));
}

void genMap()
{
    using namespace std::chrono;
    
    uint16_t eid = (robotId * 1000) + 1;
    uint64_t ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "map - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "map - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "map - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "map - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    // ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    ts = 1000;
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "map - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }
}

void genArtf()
{
    using namespace std::chrono;
    
    uint16_t eid = (robotId * 1000) + 2;
    uint64_t ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "artifact - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "artifact - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "artifact - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "artifact - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "artifact - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }
}

void genLoc()
{
    using namespace std::chrono;
    
    uint16_t eid = (robotId * 1000) + 3;
    uint64_t ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "location - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "location - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "location - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "location - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }

    slp();

    ts = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (db->upsert({db_client::DbClient::Schema(eid, ts, "location - " + std::to_string(ts))}))
    {
        std::cout << "Inserted " << eid << " with timestamp " << ts << std::endl;
    }
}

int main(int argc, char** argv)
{
    ns = std::string(argv[1]);
    if (ns == "X1")
    {
        robotId = 1;
    }
    else if (ns == "X2")
    {
        robotId = 2;
    }
    else if (ns == "X3")
    {
        robotId = 3;
    }
    else
    {
        robotId = 4;
    }
    
    db = std::unique_ptr<db_client::DbClient>(new db_client::DbClient(ns));

    std::thread mapTh(genMap);
    std::thread artfTh(genArtf);
    std::thread locTh(genLoc);

    if (mapTh.joinable())
    {
        mapTh.join();
    }

    if (artfTh.joinable())
    {
        artfTh.join();
    }
    
    if (locTh.joinable())
    {
        locTh.join();
    }

    auto res = db->selectMit();
    db_client::MIT mit = res ? res.value() : db_client::MIT();
    
    auto ids = mit.getIdKeys();
    for (auto id : ids)
    {
        std::cout << id << " : " << mit.getTimestamp(id) << std::endl;
    }
    
    // db_client::Db::Schema s;
    // s.id = 20316;
    // s.timestamp = 22110304;
    // s.data = "This is data";

    // db_client::Db::SCHEMAS v;
    // v.push_back(s);

    // std::vector<uint16_t> v1;
    // v1.push_back(20314);

    // d.upsert(v);
}    
