// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using standard messages, float 32 type
// communicate to image processing algorithm result
#include <std_msgs/Float64MultiArray.h>
#include "../../lcm_to_ros/eurecar/eurecar_v2v_test.hpp"
#include "udp_client/eurecar_v2v_test.h"
#include <std_msgs/Float64.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <lcm/lcm-cpp.hpp>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      3764						/////Eurecar
#define DF_UDP_SERVER_ADDR  "192.168.3.239"				/////Avante
#define DF_UDP_client_ADDR  "192.168.3.111"



struct timeval tv_timeo = { 0, 500000 };   // 0.5 seconds


struct struct_t_UDP
{
    int    Socket;
    struct sockaddr_in ServerAddr;
    //double   TXBuffer[DF_UDP_BUFFER_SIZE];
    double   RXBuffer[DF_UDP_BUFFER_SIZE];
};

double llh[4];

#pragma pack(push,1)
struct RX_message_data
{
    double lat;
    double lon;
    double height;
//    double tmp;

};
#pragma pack(pop)

struct struct_t_UDP           StrUDP;
struct sockaddr_in            MyAddr;
struct RX_message_data        RX_buff;

void arrayCallback(const std_msgs::Float64MultiArray& rx_msg);

// node main loop, for ROS
int main(int argc, char** argv)
{
    // node name initialization
    init(argc, argv, "udp_client");

    // assign node handler
    ros::NodeHandle nh_;

    Publisher  Avante_LLH_pub	= nh_.advertise<udp_client::eurecar_v2v_test>("/Avante/received", 100);

    //ros::Subscriber sub = nh_.subscribe("/Avante/LLH",100,arrayCallback);

/*
        //Publish
        Publisher  P_UAV	= nh_.advertise<std_msgs::Float32MultiArray>("/UAV3", 100);
        //Publisher  queue_UAV	= nh_.advertise<std_msgs::Float32MultiArray>("/queue_UAV", 100);

        //Subscribe
        ros::Subscriber sub1 = nh_.subscribe("/UAV2",100,arrayCallback);
        //ros::Subscriber sub2 = nh_.subscribe("/queue_UAV",	100,arrayCallback2);					/////UAV
*/

    // setup the loop speed, [Hz], synchronizing the hector slam loop
    ros::Rate loop_rate(5);

    //float fdt = (float)(1/20);

    // Socket Creation
    StrUDP.Socket = socket(PF_INET, SOCK_DGRAM, 0);
    if(StrUDP.Socket == -1)
    {
        printf("[ERROR] 'socket()'\n");
        return -1;
    }
    else
    {
        printf("[DONE] UDP socket is created\n");
    }

    // UDP-IP Setting_Client
    memset(&StrUDP.ServerAddr, 0, sizeof(StrUDP.ServerAddr)); // Clear to 0
    StrUDP.ServerAddr.sin_family      = PF_INET;
    StrUDP.ServerAddr.sin_port        = htons(DF_UDP_PORTNUM); // PORT#
    StrUDP.ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR); // IP for Server (Normally PC IP)

    memset(&MyAddr, 0, sizeof(MyAddr));
    MyAddr.sin_family = PF_INET;
    MyAddr.sin_port = htons(DF_UDP_PORTNUM);
    //MyAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    MyAddr.sin_addr.s_addr = inet_addr(DF_UDP_client_ADDR);

    if(bind(StrUDP.Socket,(struct sockaddr *)&MyAddr, sizeof(MyAddr))!=0)
    {
        printf("bind1() error!\n");
        return -1;
    }

    int size_addr = sizeof(StrUDP.ServerAddr);

    if(setsockopt(StrUDP.Socket,SOL_SOCKET, SO_RCVTIMEO, &tv_timeo, sizeof(tv_timeo)) == -1)
    {
        printf("setsockopt error!\n");
        return -1;
    }

    int temp_data;

    // node loop, for ROS, check ros status, ros::ok()
    //RX_buff.lat	 = llh[0];					// Avante
    //RX_buff.lon	 = llh[1];					// Avante
    //RX_buff.height = llh[2];               		// Avante

    printf("Client!\n");

    while(ros::ok())
    {
        memset(StrUDP.RXBuffer, 0, sizeof(StrUDP.RXBuffer));
        temp_data = recvfrom(StrUDP.Socket, StrUDP.RXBuffer, sizeof(StrUDP.RXBuffer), 0, (struct sockaddr *)(&StrUDP.ServerAddr), (socklen_t *)&size_addr);

        memcpy(&RX_buff, (RX_message_data*)StrUDP.RXBuffer, sizeof(RX_message_data));

        printf("recv data : %.7f  %.7f  %.7f \n",RX_buff.lat, RX_buff.lon, RX_buff.height);

        // messages
        //std_msgs::Float64MultiArray Avante_LLH_msg;
        udp_client::eurecar_v2v_test Avante_LLH_msg;


        //Avante_LLH_msg.llh.clear();
        //Avante_LLH_msg.llh.resize(4);
        Avante_LLH_msg.llh[0] = RX_buff.lat;
        Avante_LLH_msg.llh[1] = RX_buff.lon;
        Avante_LLH_msg.llh[2] = RX_buff.height;
        Avante_LLH_msg.llh[3] = 0;

        Avante_LLH_pub.publish(Avante_LLH_msg);

        // loop rate [Hz]
        loop_rate.sleep();

        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Client Avante LLH_RX\n");
    close(StrUDP.Socket);

    return 0;
}


void arrayCallback(const std_msgs::Float64MultiArray& rx_msg)
{
        llh[0] = rx_msg.data[0];		//Lat
        llh[1] = rx_msg.data[1];		//Lon
        llh[2] = rx_msg.data[2];		//Height
        llh[3] = 0;
        return;
}
