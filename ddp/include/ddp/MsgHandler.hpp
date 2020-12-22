#include <vector>
#include "ATransmitter.hpp"
#include "messages/AdvMsg.hpp"
#include "messages/AckMsg.hpp"
#include "messages/ReqMsg.hpp"
#include "messages/DataMsg.hpp"
#include "MIT.hpp"
#include "AReqHandler.hpp"
#include "ARespHandler.hpp"

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
     * @brief Handler interface that will manage ACK messages and new responses for data exchange.
     * 
     */
    ARespHandler* respHandler;

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
        reqHandler->notifyData(src, msg);
    }

    void _handleReq(ReqMsg& msg, std::string src)
    {
        std::cout << "GOT REQUEST FOR: " << msg.reqSequence << " , " << msg.reqEntryId << std::endl;
        respHandler->queueResp(msg.reqSequence, msg.reqEntryId, src);
    }

    void _handleAck(AckMsg& msg, std::string src)
    {
        if (msg.forReq)
        {
            std::cout << "GOT ACK FOR REQ" << std::endl;
            reqHandler->notifyAck(src, msg);
        }
        else
        {
            std::cout << "GOT ACK FOR DATA" << std::endl;
            respHandler->notifyAck(src, msg);
        }
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
    
    MsgHandler(ATransmitter* transmitter, AReqHandler* reqHandler, ARespHandler* respHandler) 
        : transmitter(transmitter), reqHandler(reqHandler), respHandler(respHandler) {}

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