#include <iostream>

#include "CTransceiver.hpp"

bool CTransceiver::send(uint8_t* data, uint32_t len)
{
    std::string s(data, data+len);
    std::cout << "Sending " << s << std::endl;
    return true;
};

void CTransceiver::subscribe(IObserver* observer)
{
    observers_list.push_back(observer);
};

void CTransceiver::unsubscribe(IObserver* observer)
{
    observers_list.remove(observer);
};

void CTransceiver::notify()
{
    std::cout << "There are " << observers_list.size() << " observers right now." << std::endl;

    for (std::list<IObserver*>::iterator i = observers_list.begin(); i != observers_list.end(); i++)
    {
        uint8_t data[10] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};
        (*i)->callback(data, 10);
    }
}