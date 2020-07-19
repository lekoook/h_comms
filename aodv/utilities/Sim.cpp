#include "Sim.hpp"
#include <stdint.h>

namespace aodv
{
    Sim::Sim(uint8_t numNodes, config_t configs[]) {
        this->time = 0;
        this->numNodes = numNodes;

        // Collect all events.
        for (int i=0; i<this->numNodes; i++) {
            config_t config = configs[i];
            for (int j=0; j<config.numEvents; j++) {
                this->events.push(config.events[j]);
            }
        }

        // Sort all events, repush into queue.
        int size = this->events.size();
        event_t tmp[size];
        int i = 0;
        while (!this->events.empty()) {
            tmp[i] = this->events.front();
            this->events.pop();
            i++;
        }
        std::sort(tmp,
                  tmp + size,
                  [](event_t a, event_t b) {
                      return a.time < b.time;
                  });
        for (int i=0; i<size; i++) {
            this->events.push(tmp[i]);
        }
    }

    void Sim::command(std::string command) {
        if (command == "step") {
            event_t event = this->events.front();
            if (event.time == this->time) {
                this->sends[event.eth.src].push(event.eth);
            }

            // TODO deal with receive queues
            /*
              for (int i=0; i<this->numNodes; i++) {
              this->recvs[i].push(event.eth);
              }
            */

            this->time++;
        }
    }

    void Sim::start() {
        std::string command;
        while (true) {
            std::cout << ">";
            std::cin >> command;
            this->command(command);
        }
    }

    void Sim::print_node(uint8_t i) {
        std::cout << i << "." << std::endl;
        this->print_send(i);
        this->print_recv(i);
    }

    void Sim::print_send(uint8_t i) {
        std::cout << ">";
        int size = this->sends[i].size();
        Eth eth;
        for (int i=0; i<size; i++) {
            eth = this->sends[i].front();
            this->sends[i].pop();
            print_eth(eth);
            this->sends[i].push(eth);
        }
    }

    void Sim::print_recv(uint8_t i) {
        std::cout << "<";
        int size = this->recvs[i].size();
        Eth eth;
        for (int i=0; i<size; i++) {
            eth = this->recvs[i].front();
            this->recvs[i].pop();
            print_eth(eth);
            this->recvs[i].push(eth);
        }
    }

    void Sim::print_eth(Eth eth) {
        std::string open = "(";
        std::string sep = ",";
        std::string close = ")";
        std::cout << open << eth.ttl << sep << eth.dst << sep << eth.src << sep << eth.length << close << std::endl;
    }
}
