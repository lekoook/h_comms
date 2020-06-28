#include "utilities/Sim.hpp"

int main() {
  uint8_t numNodes = 2;
  Sim::Sim sim;
  Sim::config_t configs[] =
    {
     {2,
      {
       {0, Eth::Eth()},
       {1, Eth::Eth()},
      }
     },
     {1,
      {
       {1, Eth::Eth()},
      }
     },
    };
  sim = Sim::Sim(numNodes, configs);
  sim.start();
}
