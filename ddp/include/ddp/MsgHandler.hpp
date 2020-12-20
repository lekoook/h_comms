#include <vector>
#include "ATransmitter.hpp"
#include "messages/AdvMsg.hpp"
#include "messages/AckMsg.hpp"
#include "messages/ReqMsg.hpp"
#include "messages/DataMsg.hpp"
#include "MIT.hpp"

class MsgHandler
{
private:
    /**
     * @brief Transmitter that can be used to send messages.
     * 
     */
    ATransmitter* transmitter;

    MsgType _peekType(std::vector<uint8_t>& data)
    {
        return (MsgType)data.data()[0];
    }

    void _handleAck(AckMsg& msg)
    {

    }

    void _handleAdv(AdvMsg& msg)
    {
        MIT mit;
        mit.deserialise(msg.data);

        
    }

public:
    MsgHandler() {}
    
    MsgHandler(ATransmitter* transmitter) : transmitter(transmitter) {}

    void notifyRx(std::string src, std::vector<uint8_t> data)
    {
        MsgType t = _peekType(data);
        switch (t)
        {
        case MsgType::Data:
            break;
        case MsgType::Request:
            break;
        case MsgType::Acknowledgement:
            break;
        default: // Advertisement
            AdvMsg m;
            m.deserialize(data);
            _handleAdv(m);
            break;
        }
    }
};