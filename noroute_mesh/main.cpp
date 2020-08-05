#include "node/Node.hpp"
#include <iostream>

int main()
{
    //uint8_t b[] = {'\0', '\0', '\0'};
    uint8_t l = 95;
    uint8_t b[] = {32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126};
    aodv::Node n = aodv::Node();
    std::string s = n.uint8_to_string(b, l);
    //std::cout << s << std::endl;

    uint8_t bNew[l];
    n.string_to_uint8(bNew, s);
    for (int i=0; i<l; i++) {
        //printf("%u ", bNew[i]);
        std::cout << bNew[i];
    }
    std::cout << std::endl;

    return 0;
}
