#include "lib/parson.h"
#include "utilities/Sim.hpp"

int main() {
  JSON_Value *json_root = json_parse_file("config.json");
  JSON_Array *json_configs = json_value_get_array(json_root);

  size_t numNodes = json_array_get_count(json_configs);
  size_t numEvents;
  aodv::config_t configs[numNodes];
  aodv::config_t *config;
  int time;
  uint8_t ttl, dst, src;
  double length;
  uint8_t *payload;

  JSON_Array *json_config;
  JSON_Object *json_event;
  JSON_Object *json_eth;
  for (size_t i=0; i<numNodes; i++) {
    json_config = json_array_get_array(json_configs, i);
    numEvents = json_array_get_count(json_config);
    config = (aodv::config_t*)malloc(sizeof(*config) + numEvents * sizeof(config->events[0]));
    config->numEvents = numEvents;
    for (size_t j=0; j<numEvents; j++) {
      json_event = json_array_get_object(json_config, j);
      time = json_object_get_number(json_event, "time");
      json_eth = json_object_get_object(json_event, "eth");
      ttl = json_object_get_number(json_eth, "ttl");
      dst = json_object_get_number(json_eth, "dst");
      src = json_object_get_number(json_eth, "src");
      length = json_object_get_string_len(json_eth, "payload");
      payload = (uint8_t*)json_object_get_string(json_eth, "payload");
      config->events[j] = {time, aodv::Eth(ttl, dst, src, length, payload)};
    }
    configs[i] = *config;
  }

  aodv::Sim sim = aodv::Sim(numNodes, configs);
  sim.start();

  for (long unsigned int i=0; i<sizeof(configs)/sizeof(aodv::config_t); i++) {
    free(&configs[i]);
  }
}
