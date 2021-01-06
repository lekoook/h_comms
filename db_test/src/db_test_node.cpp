#include "ros/ros.h"
#include <iostream>
#include <db_client/Db.hpp>
#include <vector>
#include <cstdint>

int main(){
    db_client::Db d("X1");
    db_client::Db::Schema s;
    s.id = 20316;
    s.timestamp = 22110304;
    s.data = "This is data";

    db_client::Db::SCHEMAS v;
    v.push_back(s);

    std::vector<uint16_t> v1;
    v1.push_back(20314);

    d.upsert(v);
}    
