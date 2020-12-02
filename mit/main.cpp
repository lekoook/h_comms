#include "MIT.hpp"
#include <iostream>

int main(void)
{
    MIT mit1 = MIT();
    MIT mit2 = MIT();
    
    mit1.update(10, 33, 100000);
    mit1.update(11, 134, 100000);

    mit2.update(10, 33, 100000);
    mit2.update(11, 134, 200000);
    mit2.update(12, 35, 100000);

    std::vector<std::pair<uint16_t, uint16_t>> requests = mit1.compare(mit2);
    for (auto item: requests)
    {
        std::cout << item.first << " : " << item.second << std::endl;
    }
    
    return 0;
}