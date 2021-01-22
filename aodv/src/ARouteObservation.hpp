#ifndef H_A_ROUTE_OBSERVATION
#define H_A_ROUTE_OBSERVATION

#include <string>

class ARouteObserver
{
public:
    virtual void notifyRouteActive() = 0;
};

class ARouteSubject
{
private:
    virtual void _notifyRouteActive(std::string destination) = 0;
public:
    virtual void subRouteValid(std::string destination, ARouteObserver* observer) = 0;
    virtual void unsubRouteValid(std::string destination, ARouteObserver* observer) = 0;
};

#endif // H_A_ROUTE_OBSERVATION