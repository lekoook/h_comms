#include "ros/ros.h"
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"
#include "TxerTh.hpp"
#include "RxerTh.hpp"

RxerTh* pRxTh; // TODO: Is there a better way than using global variable?
ros::Publisher rxPubber;
std::string robotAddr;

void rxCb(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data)
{
    if (_srcAddress != robotAddr)
    {
        while(!pRxTh->recvOne(RxQueueData(_data, _srcAddress, _dstAddress)));
    }
}

bool txData(ptp_comms::TxData::Request &req, ptp_comms::TxData::Response &res, TxerTh* th)
{
    if (req.dest != robotAddr)
    {
        res.successful = th->sendOne(req.data, req.dest);
    }
    else
    {
        res.successful = false;
    }
    
    return res.successful;
}

void pubRx(std::string src, std::vector<uint8_t> data)
{
    ptp_comms::RxData msg;
    msg.data = data;
    msg.src = src;
    rxPubber.publish(msg);
}

int main(int argc, char** argv)
{
    // Constants
    const double PING_INTERVAL = 1.0;

    robotAddr = argv[1];

    ros::init(argc, argv, "point_to_point_comms");
    ros::NodeHandle nh;
    rxPubber = nh.advertise<ptp_comms::RxData>("rx_data", 100);
    
    subt::CommsClient cc(robotAddr);
    TxerTh txTh(&cc);
    std::function<void(std::string, std::vector<uint8_t>)> cb = &pubRx;
    RxerTh rxTh(&cc, &txTh, pubRx);
    pRxTh = &rxTh;

    cc.Bind(rxCb, robotAddr);
    cc.StartBeaconInterval(ros::Duration(PING_INTERVAL));
    txTh.start();
    rxTh.start();

    ros::ServiceServer txService = nh.advertiseService<ptp_comms::TxData::Request, ptp_comms::TxData::Response>(
        "tx_data", 
        std::bind(&txData, std::placeholders::_1, std::placeholders::_2, &txTh)
        );

    ros::spin();
}