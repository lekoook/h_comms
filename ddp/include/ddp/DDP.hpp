#include <thread>
#include <vector>
#include "ptp_comms/TxData.h"
#include "ptp_comms/RxData.h"

class DDP
{
private:
    ros::ServiceClient msgSrvC;
    ros::Subscriber msgSubber;
    std::thread mainTh;
    std::atomic<bool> mainRunning;

    void _subCb(const ptp_comms::RxData::ConstPtr& msg)
    {
        ROS_INFO("src: %s", msg->src.c_str());
        ROS_INFO("data[0]: %u", msg->data[0]);
    }

    void _runMain()
    {
        // Main thread execution.
        while(mainRunning.load())
        {
            ptp_comms::TxData msg;
            msg.request.data = std::vector<uint8_t> {65, 66, 67};
            msg.request.dest = "X2";
            if (msgSrvC.call(msg))
            {
                ROS_INFO("Call success");
            }
            else
            {
                ROS_INFO("Call failed");
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

public:
    DDP(ros::NodeHandle& nh)
    {
        msgSrvC = nh.serviceClient<ptp_comms::TxData>("tx_data");
        msgSubber = nh.subscribe<ptp_comms::RxData>("rx_data", 100, &DDP::_subCb, this);

        // Begin main thread operation.
        mainRunning.store(true);
        mainTh = std::thread(&DDP::_runMain, this);
    }


};