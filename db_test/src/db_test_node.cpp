#include <iostream>
#include <db_client/Db.hpp>
#include <vector>
#include <cstdint>

int main(){
    Db d;
    Db::Schema s;
    s.id = 20314;
    s.timestamp = 22110304;
    s.data = "This is data";

    Db::SCHEMAS v;
    v.push_back(s);

    std::vector<uint16_t> v1;
    v1.push_back(20314);

    d.insert(v);
}    