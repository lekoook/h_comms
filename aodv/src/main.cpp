#include "ros/ros.h"
#include "aodv/send_map.h"
#include "nav_msgs/OccupancyGrid.h"

class AODV
{
    private:
        ros::NodeHandle n;
        ros::ServiceServer map_service;
        ros::Publisher map_pub;
        

    public:
        AODV(ros::NodeHandle& nh) : n(nh) {
            // Service servers - ROS API to allow other nodes to send data
            start_map_service();

            // Port binding - Receive information from bound ports and send them to the appropriate ROS topics
            // Comms client for 'X1' robot.
            // subt::CommsClient x1Client = subt::CommsClient("X1");
            // x1Client.Bind(handlePacket, "X1");

        }
        ~AODV() {}

        void sendMap(aodv::send_map::Request &req, aodv::send_map::Response &res){
            ROS_INFO("[SEND_MAP] Requested");
            // Serialise map and send to AODV layer
            // Await response - maybe need to time out somewhere
            ROS_INFO("[SEND_MAP] Response: %d", res.response);
        }

        void start_map_service(){
            // map_service = n.advertiseService("send_map", sendMap);
            map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid",1);
        }

        // void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        //      // Deserialise the received data
        //      // Identify type of packet (e.g RREP, RREQ)
        //      // Case for each type
        // }
        
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AODV_API");
	ros::NodeHandle nh;
	AODV server(nh);
    ROS_INFO("Started AODV node");
}