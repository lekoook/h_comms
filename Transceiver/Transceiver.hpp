#include <iostream>

class IObserver
{
public:
    virtual void callback(uint8_t* data, uint32_t len) = 0;
};

class IObservable
{
public:
    virtual void subscribe(IObserver* observer) = 0;
    virtual void unsubscribe(IObserver* observer) = 0;
    virtual void notify() = 0;
};

class Transceiver : public IObservable
{
public:
    virtual bool send(uint8_t* data, uint32_t len) = 0;
};