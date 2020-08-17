#include "Node.hpp"
#include <iostream>

namespace aodv
{
    Node::Node() :
        id(), addr(), broadcastAddr()
    {
    }

    Node::Node(uint32_t id, std::string addr, std::string broadcastAddr) :
        id(id), addr(addr), broadcastAddr(broadcastAddr)
    {
    }

    std::string Node::send(Eth &eth, bool isOriginating)
    {
        if (isOriginating) {
            // Overwrite seq and src, because this node is originating eth.
            eth.seq = this->seq;
            eth.src = this->addr;
            this->seq++;
        }

        // Where seg is a segment, aodv::MAX_MESSAGE_SIZE/2 == aodv::ETH_NONVAR_LEN + seg.srcLength + seg.dstLength + seg.payloadLength
        // Where eth is a packet, there are sizeof(typeof(eth.segSeqMax)) segments.
        // So the maximum eth.payloadLength must be:
        const uint16_t maxPayloadLength = aodv::MAX_MESSAGE_SIZE/2 - (aodv::ETH_NONVAR_LEN + eth.srcLength + eth.dstLength);
        assert (eth.payloadLength <= sizeof(uint32_t) * maxPayloadLength);
        if (eth.payloadLength < maxPayloadLength) {
            eth.segSeqMax = 1;
        } else {
            eth.segSeqMax = (eth.payloadLength / maxPayloadLength) + ((eth.payloadLength % maxPayloadLength) != 0);
        }

        // Overwrite seq and src, because this node is originating eth.
        eth.seq = this->seq;
        eth.src = this->addr;
        this->seq++;

        // Split packet into segments.
        int segSeq=0, p=maxPayloadLength;
        for (; p<eth.payloadLength; segSeq++,p+=maxPayloadLength) {
            aodv::Eth seg(eth);
            seg.payloadLength = maxPayloadLength;
            seg.payload = eth.payload.substr(p - maxPayloadLength, seg.payloadLength);
            seg.segSeq = segSeq;

            uint16_t length = aodv::ETH_NONVAR_LEN + seg.srcLength + seg.dstLength + seg.payloadLength;
            uint8_t msg[length];
            std::cout << "SENDING" << std::endl
                // << seg.seq << std::endl
                // << seg.dst << std::endl
                // << seg.dstLength << std::endl
                // << seg.src << std::endl
                // << seg.srcLength << std::endl
                << seg.payloadLength << std::endl
                << seg.payload << std::endl;
            seg.serialise(msg);
        }

        aodv::Eth seg(eth);
        seg.payloadLength = eth.payloadLength - (p - maxPayloadLength);
        seg.payload = eth.payload.substr(p - maxPayloadLength, seg.payloadLength);
        seg.segSeq = segSeq;

        uint16_t length = aodv::ETH_NONVAR_LEN + eth.srcLength + eth.dstLength + eth.payloadLength;
        uint8_t msg[length];

        std::cout << "SENDING" << std::endl
            // << seg.seq << std::endl
            // << seg.dst << std::endl
            // << seg.dstLength << std::endl
            // << seg.src << std::endl
            // << seg.srcLength << std::endl
            << seg.payloadLength << std::endl
            << seg.payload << std::endl;
        seg.serialise(msg);
        return uint8_to_string(msg, length);
    }

    tl::optional<aodv::Eth> Node::receive(std::string data)
    {
        aodv::Eth seg = aodv::Eth();
        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        seg.deserialise(msg);

        std::cout << "RECVING" << std::endl
            // << seg.seq << std::endl
            // << seg.dst << std::endl
            // << seg.dstlength << std::endl
            // << seg.src << std::endl
            // << seg.srclength << std::endl
            << seg.payloadLength << std::endl
            << seg.payload << std::endl;

        if (seg.src == this->addr) {
            std::cout << "ignore packet from myself" << std::endl;
            return tl::nullopt;
        }

        if (seg.dst == this->addr) {
            auto search = this->tableDesegment.find(seg.seq);
            if (search == tableDesegment.end()) {
                // No segments from this sequence number exist in tableDesegment.
                tableDesegment[seg.seq] = std::vector<aodv::Eth>(seg.segSeqMax, aodv::Eth());
                tableDesegment[seg.seq][seg.segSeq] = seg;

            } else {
                if (tableDesegment[seg.seq][seg.segSeq] == aodv::Eth()) {
                    tableDesegment[seg.seq][seg.segSeq] = seg;
                } else {
                    bool flag = true;
                    for (aodv::Eth seg : tableDesegment[seg.seq]) {
                        if (seg == aodv::Eth()) {
                            flag = false;
                        }
                    }

                    if (flag) {
                        // All segments have been received, perform desegmentation.
                        // aodv::Eth seg;
                        uint64_t payloadLengthTotal = 0;
                        for (aodv::Eth seg : tableDesegment[seg.seq]) {
                            payloadLengthTotal += seg.payloadLength;
                        }


                        // Copy all the segments payload into one final payload according to order.
                        // uint8_t* payload = (uint8_t*)malloc(payloadLengthTotal);
                        // memcpy(payload, tableDesegment[seg.seq][0].payload, tableDesegment[seg.seq][0].payloadLength);
                        // for (uint32_t i=1; i<seg.segSeqMax; i++) {
                        //     memcpy(payload + tableDesegment[seg.seq][i-1].payloadLength, tableDesegment[seg.seq][i].payload, tableDesegment[seg.seq][i].payloadLength);
                        // }

                        std::string payload = "";
                        payload.reserve(payloadLengthTotal);
                        for (int i = 0; i < seg.segSeqMax; i++)
                        {
                            payload += tableDesegment[seg.seq][i].payload;
                        }

                        // Packet must consist of at least one segment. Copy fields of first segment.
                        aodv::Eth eth = aodv::Eth(tableDesegment[seg.seq][0]);
                        eth.payloadLength = payloadLengthTotal;
                        eth.payload = payload;
                        return eth; // return optional object that contains eth.
                    }

                }
            }

        } else {
            auto search = this->tableAddr.find(seg.src);
            if (search == this->tableAddr.end()) {
                // No segments from this address exist in tableAddr.
                this->tableAddr[seg.src] = std::unordered_map<uint32_t, std::vector<bool>>();
                this->tableAddr[seg.src][seg.seq] = std::vector<bool>(seg.segSeqMax);
                this->tableAddr[seg.src][seg.seq][seg.segSeq] = true;
                this->send(seg, false);

            } else {
                std::unordered_map<uint32_t, std::vector<bool>> tableSeq = search->second;
                auto search2 = tableSeq.find(seg.seq);
                if (search2 == tableSeq.end()) {
                    // No segments from this sequence number exist in tableSeq.
                    tableSeq[seg.seq] = std::vector<bool>(seg.segSeqMax);
                    tableSeq[seg.seq][seg.segSeq] = true;
                    this->send(seg, false);

                } else {
                    if (!tableSeq[seg.seq][seg.segSeq]) {
                        // Segment was not already forwarded.
                        tableSeq[seg.seq][seg.segSeq] = true;
                        this->send(seg, false);
                    }
                }
            }
        }

        return tl::nullopt; // return optional object that does not contain a value.
    }

    std::string Node::uint8_to_string(uint8_t b[], std::string::size_type l)
    {
        /*
         * A uint8_t has bits denoted as: abcdefgh.
         */
        char c;
        uint8_t s[l*2 + 1]; // + 1 for '\0'
        std::string::size_type i=0;
        for (; i<l; i++) {
            c = 0b10101010u;
            c |= (b[i] >> 7) << 6;
            c |= ((b[i] >> 6) & 1) << 4;
            c |= ((b[i] >> 5) & 1) << 2;
            c |= (b[i] >> 4) & 1;
            s[2*i] = c;
            c = 0b10101010u;
            c |= ((b[i] >> 3) & 1) << 6;
            c |= ((b[i] >> 2) & 1) << 4;
            c |= ((b[i] >> 1) & 1) << 2;
            c |= b[i] & 1;
            s[2*i+1] = c;
        }
        s[2*i] = '\0';
        return std::string((char*)s,l*2+1);
    }

    void Node::string_to_uint8(uint8_t b[], std::string s)
    {
        /*
         * Denotation of bits is as in the body of uint8_to_string(uint8_t b[], std::string::size_type l).
         */
        for (std::string::size_type i=0; i<s.size(); i+=2) {
            b[i/2] = ((s[i] & 0b01000000u) << 1) | ((s[i] & 0b00010000u) << 2) | ((s[i] & 0b00000100u) << 3) | ((s[i] & 1) << 4);
            b[i/2] |= ((s[i+1] & 0b01000000u) >> 3) | ((s[i+1] & 0b00010000u) >> 2) | ((s[i+1] & 0b00000100u) >> 1) | (s[i+1] & 1);
        }
    }
}
