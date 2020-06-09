#include <iostream>
#include "CTransceiver.hpp"

class Observer : public IObserver
{
public:
    Observer(int id)
    {
        this->id = id;
    }

    void callback(uint8_t* data, uint32_t len)
    {
        std::cout << "I am " << id << " and I received " << std::string(data, data + len) << std::endl;
    }

private:
    int id;
};

int main(void)
{
    Observer* subber0 = new Observer(0);
    Observer* subber1 = new Observer(1);
    Observer* subber2 = new Observer(2);
    Observer* subber3 = new Observer(3);

    Transceiver* publisher = new CTransceiver();
    publisher->subscribe(subber0);
    publisher->subscribe(subber1);
    publisher->subscribe(subber2);
    publisher->subscribe(subber3);

    publisher->notify(); // Notify all 4 subscribers.

    publisher->unsubscribe(subber1); // Unsubscribe 1 and 3.
    publisher->unsubscribe(subber3);

    publisher->notify(); // Only 0 and 2 should be notified now.
}