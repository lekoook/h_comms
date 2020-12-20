#include <string>
#include <vector>
#include <functional>

class ATransceiver
{
protected:
    std::function<void(std::string, std::vector<uint8_t>&)> rxCb;
public:
    virtual bool transmit(std::string dest, std::vector<uint8_t>& data);
};
