#include <list>
#include "Transceiver.hpp"

class CTransceiver : public Transceiver
{
public:
    bool send(uint8_t* data, uint32_t len);
    void subscribe(IObserver* observer);
    void unsubscribe(IObserver* observer);
    void notify();

private:
    std::list<IObserver*> observers_list;
};