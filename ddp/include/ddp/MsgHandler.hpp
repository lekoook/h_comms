#include <vector>
#include "ATransmitter.hpp"
#include "messages/AdvMsg.hpp"
#include "messages/AckMsg.hpp"
#include "messages/ReqMsg.hpp"
#include "messages/DataMsg.hpp"
#include "MIT.hpp"
#include "AReqHandler.hpp"

class MsgHandler
{
private:
    /**
     * @brief Transmitter that can be used to send messages.
     * 
     */
    ATransmitter* transmitter;

    /**
     * @brief Handler interface that will manage ACK, DATA messages and new requests for data exchange.
     * 
     */
    AReqHandler* reqHandler;

    /**
     * @brief Peeks at the type of message the bytes vector contains.
     * 
     * @param data Bytes vector.
     * @return MsgType Type of message.
     */
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

        // TODO: Compare incoming MIT with our own local one.

        // Some example request
        reqHandler->queueReq(2034, "X3");
    }

public:
    MsgHandler() {}
    
    MsgHandler(ATransmitter* transmitter, AReqHandler* reqHandler) : transmitter(transmitter), reqHandler(reqHandler) {}

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