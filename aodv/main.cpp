#include "utilities/Sim.hpp"

int main() {
  uint8_t numNodes = 2;
  int numEvents;

  numEvents = 2;
  aodv::config_t *config1 = (aodv::config_t*)malloc(sizeof(*config1) + numEvents * sizeof(config1->events[0]));
  config1->numEvents = numEvents;
  config1->events[0] = {0, aodv::Eth()};
  config1->events[1] = {1, aodv::Eth()};

  numEvents = 1;
  aodv::config_t *config2 = (aodv::config_t*)malloc(sizeof(*config2) + numEvents * sizeof(config2->events[0]));
  config2->numEvents = numEvents;
  config2->events[0] = {1, aodv::Eth()};

  aodv::config_t configs[] = {*config1, *config2};
  aodv::Sim sim = aodv::Sim(numNodes, configs);
  sim.start();

  for (long unsigned int i=0; i<sizeof(configs)/sizeof(aodv::config_t); i++) {
    free(&configs[i]);
  }
}
