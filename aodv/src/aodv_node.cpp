#include <iostream>
#include "messages/RreqMsg.hpp"
#include "messages/RrepMsg.hpp"
#include "messages/RrerMsg.hpp"
#include "RouteTable.hpp"

// #define SHOW_MORE

void test(aodv::RreqMsg& msg)
{
    std::cout << "=== Testing ===" << std::endl;
    auto ser = msg.serialize();
    aodv::RreqMsg msg2;
    msg2.deserialize(ser);

    #ifdef SHOW_MORE
    std::cout << "--- Original: " << std::endl;
    std::cout << "isJoin       : " << ((msg.isJoin) ? "true" : "false") << std::endl;
    std::cout << "isRepair     : " << ((msg.isRepair) ? "true" : "false") << std::endl;
    std::cout << "isGratuitous : " << ((msg.isGratuitous) ? "true" : "false") << std::endl;
    std::cout << "isDestOnly   : " << ((msg.isDestOnly) ? "true" : "false") << std::endl;
    std::cout << "isUnkSequence: " << ((msg.isUnkSequence) ? "true" : "false") << std::endl;
    std::cout << "Hop Count    : " << (int)msg.hopCount << std::endl;
    std::cout << "RREQ ID      : " << msg.rreqId << std::endl;
    std::cout << "Dest Addr    : " << msg.getDestAddr() << std::endl;
    std::cout << "Dest Seq     : " << msg.destSeq << std::endl;
    std::cout << "Src Addr     : " << msg.getSrcAddr() << std::endl;
    std::cout << "Src Seq      : " << msg.srcSeq << std::endl;
    std::cout << "--- Serialized: " << std::endl;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%hX\t", (ser[(i * 4) + j]));
        }
        std::cout << std::endl;
    }

    std::cout << "--- Reconstructed: " << std::endl;
    std::cout << "isJoin       : " << ((msg2.isJoin) ? "true" : "false") << std::endl;
    std::cout << "isRepair     : " << ((msg2.isRepair) ? "true" : "false") << std::endl;
    std::cout << "isGratuitous : " << ((msg2.isGratuitous) ? "true" : "false") << std::endl;
    std::cout << "isDestOnly   : " << ((msg2.isDestOnly) ? "true" : "false") << std::endl;
    std::cout << "isUnkSequence: " << ((msg2.isUnkSequence) ? "true" : "false") << std::endl;
    std::cout << "Hop Count    : " << (int)msg2.hopCount << std::endl;
    std::cout << "RREQ ID      : " << msg2.rreqId << std::endl;
    std::cout << "Dest Addr    : " << msg2.getDestAddr() << std::endl;
    std::cout << "Dest Seq     : " << msg2.destSeq << std::endl;
    std::cout << "Src Addr     : " << msg2.getSrcAddr() << std::endl;
    std::cout << "Src Seq      : " << msg2.srcSeq << std::endl;
    #endif // SHOW_MORE

    std::cout << "--- Result: " << std::endl;
    std::cout << "isJoin       : " << ((msg2.isJoin == msg.isJoin) ? "OK" : "ERROR") << std::endl;
    std::cout << "isRepair     : " << ((msg2.isRepair == msg.isRepair) ? "OK" : "ERROR") << std::endl;
    std::cout << "isGratuitous : " << ((msg2.isGratuitous == msg.isGratuitous) ? "OK" : "ERROR") << std::endl;
    std::cout << "isDestOnly   : " << ((msg2.isDestOnly == msg.isDestOnly) ? "OK" : "ERROR") << std::endl;
    std::cout << "isUnkSequence: " << ((msg2.isUnkSequence == msg.isUnkSequence) ? "OK" : "ERROR") << std::endl;
    std::cout << "Hop Count    : " << ((msg2.hopCount == msg.hopCount) ? "OK" : "ERROR") << std::endl;
    std::cout << "RREQ ID      : " << ((msg2.rreqId == msg.rreqId) ? "OK" : "ERROR") << std::endl;
    std::cout << "Dest Addr    : " << ((msg2.getDestAddr() == msg.getDestAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "Dest Seq     : " << ((msg2.destSeq == msg.destSeq) ? "OK" : "ERROR") << std::endl;
    std::cout << "Src Addr     : " << ((msg2.getSrcAddr() == msg.getSrcAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "Src Seq      : " << ((msg2.srcSeq == msg.srcSeq) ? "OK" : "ERROR") << std::endl;
    std::cout << "=== Testing End ===" << std::endl;
}

void test(aodv::RrepMsg& msg)
{
    std::cout << "=== Testing ===" << std::endl;
    auto ser = msg.serialize();
    aodv::RrepMsg msg2;
    msg2.deserialize(ser);

    #ifdef SHOW_MORE
    std::cout << "--- Original: " << std::endl;
    std::cout << "isRepair     : " << ((msg.isRepair) ? "true" : "false") << std::endl;
    std::cout << "isAckRequired: " << ((msg.isAckRequired) ? "true" : "false") << std::endl;
    std::cout << "prefixSize   : " << ((msg.getPrefixSize()) ? "true" : "false") << std::endl;
    std::cout << "Hop Count    : " << (int)msg.hopCount << std::endl;
    std::cout << "Dest Addr    : " << msg.getDestAddr() << std::endl;
    std::cout << "Dest Seq     : " << msg.destSeq << std::endl;
    std::cout << "Src Addr     : " << msg.getSrcAddr() << std::endl;
    std::cout << "Lifetime     : " << msg.lifetime << std::endl;
    std::cout << "--- Serialized: " << std::endl;
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%hX\t", (ser[(i * 4) + j]));
        }
        std::cout << std::endl;
    }

    std::cout << "--- Reconstructed: " << std::endl;
    std::cout << "isRepair     : " << ((msg2.isRepair) ? "true" : "false") << std::endl;
    std::cout << "isAckRequired: " << ((msg2.isAckRequired) ? "true" : "false") << std::endl;
    std::cout << "prefixSize   : " << ((msg2.getPrefixSize()) ? "true" : "false") << std::endl;
    std::cout << "Hop Count    : " << (int)msg2.hopCount << std::endl;
    std::cout << "Dest Addr    : " << msg2.getDestAddr() << std::endl;
    std::cout << "Dest Seq     : " << msg2.destSeq << std::endl;
    std::cout << "Src Addr     : " << msg2.getSrcAddr() << std::endl;
    std::cout << "Lifetime     : " << msg2.lifetime << std::endl;
    #endif // SHOW_MORE

    std::cout << "--- Result: " << std::endl;
    std::cout << "isRepair     : " << ((msg2.isRepair == msg.isRepair) ? "OK" : "ERROR") << std::endl;
    std::cout << "isAckRequired: " << ((msg2.isAckRequired == msg.isAckRequired) ? "OK" : "ERROR") << std::endl;
    std::cout << "prefixSize   : " << ((msg2.getPrefixSize() == msg.getPrefixSize()) ? "OK" : "ERROR") << std::endl;
    std::cout << "Hop Count    : " << ((msg2.hopCount == msg.hopCount) ? "OK" : "ERROR") << std::endl;
    std::cout << "Dest Addr    : " << ((msg2.getDestAddr() == msg.getDestAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "Dest Seq     : " << ((msg2.destSeq == msg.destSeq) ? "OK" : "ERROR") << std::endl;
    std::cout << "Src Addr     : " << ((msg2.getSrcAddr() == msg.getSrcAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "Lifetime     : " << ((msg2.lifetime == msg.lifetime) ? "OK" : "ERROR") << std::endl;
    std::cout << "=== Testing End ===" << std::endl;
}

void test(aodv::RrerMsg& msg)
{
    std::cout << "=== Testing ===" << std::endl;
    auto ser = msg.serialize();
    aodv::RrerMsg msg2;
    msg2.deserialize(ser);

    #ifdef SHOW_MORE
    std::cout << "--- Original: " << std::endl;
    std::cout << "isNoDelete     : " << ((msg.isNoDelete) ? "true" : "false") << std::endl;
    std::cout << "Dest Count     : " << (int)msg.getDestCount() << std::endl;
    std::cout << "UnrDest Addr   : " << msg.getUnrDestAddr() << std::endl;
    std::cout << "UnrDest Seq    : " << msg.unrDestSeq << std::endl;
    std::cout << "AddUnrDest Addr: " << msg.getAddUnrDestAddr() << std::endl;
    std::cout << "AddUnrDest Seq : " << msg.addUnrDestSeq << std::endl;
    std::cout << "--- Serialized: " << std::endl;
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%hX\t", (ser[(i * 4) + j]));
        }
        std::cout << std::endl;
    }

    std::cout << "--- Reconstructed: " << std::endl;
    std::cout << "isNoDelete     : " << ((msg2.isNoDelete) ? "true" : "false") << std::endl;
    std::cout << "Dest Count     : " << (int)msg2.getDestCount() << std::endl;
    std::cout << "UnrDest Addr   : " << msg2.getUnrDestAddr() << std::endl;
    std::cout << "UnrDest Seq    : " << msg2.unrDestSeq << std::endl;
    std::cout << "AddUnrDest Addr: " << msg2.getAddUnrDestAddr() << std::endl;
    std::cout << "AddUnrDest Seq : " << msg2.addUnrDestSeq << std::endl;
    #endif // SHOW_MORE

    std::cout << "--- Result: " << std::endl;
    std::cout << "isNoDelete     : " << ((msg2.isNoDelete == msg.isNoDelete) ? "OK" : "ERROR") << std::endl;
    std::cout << "Dest Count     : " << ((msg2.getDestCount() == msg.getDestCount()) ? "OK" : "ERROR") << std::endl;
    std::cout << "UnrDest Addr   : " << ((msg2.getUnrDestAddr() == msg.getUnrDestAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "UnrDest Seq    : " << ((msg2.unrDestSeq == msg.unrDestSeq) ? "OK" : "ERROR") << std::endl;
    std::cout << "AddUnrDest Addr: " << ((msg2.getAddUnrDestAddr() == msg.getAddUnrDestAddr()) ? "OK" : "ERROR") << std::endl;
    std::cout << "AddUnrDest Seq : " << ((msg2.addUnrDestSeq == msg.addUnrDestSeq) ? "OK" : "ERROR") << std::endl;
    std::cout << "=== Testing End ===" << std::endl;
}

int main(int argc, char** argv)
{
    // aodv::RreqMsg msg1(3, "X2", 2, "X1", 1, true, true, true, true, true);
    // aodv::RreqMsg msg2(3, "X2", 2, "X1", 1, true, true, true, true, false);
    // aodv::RreqMsg msg3(3, "X2", 2, "X1", 1, true, true, true, false, true);
    // aodv::RreqMsg msg4(3, "X2", 2, "X1", 1, true, true, false, true, true);
    // aodv::RreqMsg msg5(3, "X2", 2, "X1", 1, true, false, true, true, true);
    // aodv::RreqMsg msg6(3, "X2", 2, "X1", 1, false, true, true, true, true);
    // aodv::RreqMsg msg7(1, "X2", 2, "X3", 3, true, true, true, true, true);
    // aodv::RreqMsg msg8(2, "X2", 1, "X3", 3, true, true, true, true, true);
    // aodv::RreqMsg msg9(3, "X2", 1, "X3", 2, true, true, true, true, true);
    // test(msg1);
    // test(msg2);
    // test(msg3);
    // test(msg4);
    // test(msg5);
    // test(msg6);
    // test(msg7);
    // test(msg8);
    // test(msg9);
    // aodv::RrepMsg msg1("X4", 2, "X1", 1, 31, true, true);
    // aodv::RrepMsg msg2("X4", 2, "X1", 1, 31, true, true);
    // aodv::RrepMsg msg3("X4", 2, "X1", 1, 31, true, true);
    // aodv::RrepMsg msg4("X4", 2, "X1", 1, 31, true, true);
    // aodv::RrepMsg msg5("X2", 2, "X1", 1, 31, true, false);
    // aodv::RrepMsg msg6("X2", 2, "X1", 1, 31, false, true);
    // aodv::RrepMsg msg7("X2", 2, "X3", 3, 31, true, true);
    // aodv::RrepMsg msg8("X2", 1, "X3", 3, 31, true, true);
    // aodv::RrepMsg msg9("X2", 1, "X3", 2, 31, true, true);
    // aodv::RrerMsg msg1("X4", 2, 1, true);
    // aodv::RrerMsg msg2("X4", 2, 1, true);
    // aodv::RrerMsg msg3("X4", 2, 1, true);
    // aodv::RrerMsg msg4("X4", 2, 1, true);
    // aodv::RrerMsg msg5("X2", 2, "X1", 1, 31, false);
    // aodv::RrerMsg msg6("X2", 2, "X1", 1, 31, true);
    // aodv::RrerMsg msg7("X2", 2, "X3", 3, 31, true);
    // aodv::RrerMsg msg8("X2", 1, "X3", 3, 31, true);
    // aodv::RrerMsg msg9("X2", 1, "X3", 2, 31, true);
    // test(msg1);
    // test(msg2);
    // test(msg3);
    // test(msg4);
    // test(msg5);
    // test(msg6);
    // test(msg7);
    // test(msg8);
    // test(msg9);

    aodv::RouteTable rt;
    aodv::RouteTableEntry entry;

    entry.destination = "X1";
    entry.destSequence = 1;
    entry.nextHop = "X2";
    entry.hopCount = 1;
    entry.lifetime = 1000;
    entry.isValidRoute = true;
    entry.precursors = {"X2", "X3"};
    rt.upsertEntry(entry);

    entry.destination = "X2";
    entry.destSequence = 2;
    entry.nextHop = "X3";
    entry.hopCount = 2;
    entry.lifetime = 10000;
    entry.isValidRoute = false;
    entry.precursors = {"X1", "X3"};
    rt.upsertEntry(entry);

    entry.destination = "X1";
    entry.destSequence = 1;
    entry.nextHop = "X2";
    entry.hopCount = 1;
    entry.lifetime = 1000;
    entry.isValidRoute = false;
    entry.precursors = {"X2", "X3"};
    rt.upsertEntry(entry);

    // entry.destination = "X1";
    // entry.destSequence = 1;
    // entry.nextHop = "X2";
    // entry.hopCount = 1;
    // entry.lifetime = 1000;
    // entry.isValidRoute = true;
    // entry.precursors = {"X2", "X3"};
    // rt.upsertEntry(entry);

    std::cout << rt.isValidRoute("X1") << std::endl;
    std::cout << rt.entryExists("X2") << std::endl;
    std::cout << rt.entryExists("X3") << std::endl;
    std::cout << rt.getEntry("X2", &entry) << " " << entry.destination << std::endl;
}