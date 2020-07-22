#include "node/Node.hpp"
#include <iostream>

int main()
{
    // uint8_t b[] = {'\0', '\0', '\0', '\0'}; // 84 21 08 42 10 10000100 00100001 00001000 01000010 00010000
    uint8_t b[] = {122, 121, 120, 119};        // be af 9b e2 f7 10111110 10101111 10011011 11100010 11110111
    aodv::Node n = aodv::Node();
    std::string s = n.uint8_to_string(b, 4);
    std::cout << s << std::endl;

    uint8_t bNew[4];
    n.string_to_uint8(bNew, s); // 00 7a 69 f8 00000000 01111010 01101001 11111000
    std::cout << bNew[0] << bNew[1] << bNew[2] << bNew[3] << std::endl;

    return 0;
}
