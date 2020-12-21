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

    void _handleData(DataMsg& msg, std::string src)
    {
        std::cout << "GOT DATA" << std::endl;
    }

    void _handleReq(ReqMsg& msg, std::string src)
    {
        std::cout << "GOT REQUEST" << std::endl;
    }

    void _handleAck(AckMsg& msg, std::string src)
    {
        std::cout << "GOT ACK" << std::endl;
    }

    void _handleAdv(AdvMsg& msg, std::string src)
    {
        std::cout << "GOT ADV" << std::endl;
        MIT mit;
        mit.deserialise(msg.data);

        // TODO: Compare incoming MIT with our own local one.

        // Some example request
        MIT mock;
        auto v = mock.compare(mit);
        reqHandler->queueReq(mit.convertToEntryId(v[0].first, v[0].second), src);
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
        {
            DataMsg dataM;
            dataM.deserialize(data);
            _handleData(dataM, src);
            break;
        }
        case MsgType::Request:
        {
            ReqMsg req;
            req.deserialize(data);
            _handleReq(req, src);
            break;
        }
        case MsgType::Acknowledgement:
        {
            AckMsg ack;
            ack.deserialize(data);
            _handleAck(ack, src);
            break;
        }
        default: // Advertisement
        {
            AdvMsg adv;
            adv.deserialize(data);
            _handleAdv(adv, src);
            break;
        }
        }
    }
};