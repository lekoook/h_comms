#include "include/json.hpp"
#include <iostream>

int main()
{
    std::string s = "te\\st";
    const uint8_t* u = reinterpret_cast<const uint8_t*>(&s[0]);
    nlohmann::json obj;
    obj["s"] = s;
    obj["u"] = (char*)u;
    std::string serialisedData = obj.dump();
    std::cout << serialisedData << std::endl;
}
