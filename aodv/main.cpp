#include "utilities/Sim.hpp"

int main() {
  uint8_t numNodes = 2;
  aodv::config_t configs[] =
    {
     {2,
      {
       {0, aodv::Eth::Eth()},
       {1, aodv::Eth::Eth()},
      }
     },
     {1,
      {
       {1, aodv::Eth::Eth()},
      }
     },
    };
  aodv::Sim sim = aodv::Sim::Sim(numNodes, configs);
  sim.start();
}
