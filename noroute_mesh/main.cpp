#include "node/Node.hpp"
#include <iostream>

int main()
{
    //uint8_t b[] = {'\0', '\0', '\0'};
    uint8_t b[] = {122, 121, 120};
    aodv::Node n = aodv::Node();
    std::string s = n.uint8_to_string(b, 3);
    std::cout << s << std::endl;

    uint8_t bNew[3];
    n.string_to_uint8(bNew, s);
    std::cout << bNew[0] << bNew[1] << bNew[2] << std::endl;

    return 0;
}
