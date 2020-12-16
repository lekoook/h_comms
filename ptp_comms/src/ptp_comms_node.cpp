#include "ros/ros.h"
#include "ptp_comms/TxData.h"
#include "TxerTh.hpp"
#include "RxerTh.hpp"

RxerTh* pRxTh; // TODO: Is there a better way than using global variable?

void rxCb(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
{
    pRxTh->recvOne(RxQueueData(_data, _srcAddress));
}

bool txData(ptp_comms::TxData::Request &req, ptp_comms::TxData::Response &res, TxerTh* th)
{
    res.successful = th->sendOne(TxQueueData(req.data, req.dest));
    return res.successful;
}

int main(int argc, char** argv)
{
    // Constants
    const double PING_INTERVAL = 1.0;

    std::string robotAddr = argv[1];

    ros::init(argc, argv, robotAddr + "_point_to_point_comms");
    ros::NodeHandle nh;
    
    subt::CommsClient cc(robotAddr);
    TxerTh txTh(&cc);
    RxerTh rxTh(&cc);
    pRxTh = &rxTh;
    

    cc.Bind(rxCb, robotAddr);
    cc.StartBeaconInterval(ros::Duration(PING_INTERVAL));
    txTh.start();
    rxTh.start();

    ros::ServiceServer txService = nh.advertiseService<ptp_comms::TxData::Request, ptp_comms::TxData::Response>(
        robotAddr + "/tx_data", 
        std::bind(&txData, std::placeholders::_1, std::placeholders::_2, &txTh)
        );

    ros::spin();
}