#define UAV_NUM1

// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using standard messages, float 32 type
// communicate to image processing algorithm result
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      3764						
#define DF_UDP_SERVER_ADDR  "192.168.3.111"	//"127.0.0.1" -> local	
#define DF_UDP_client_ADDR  "192.168.3.51"	//"127.0.0.1" -> local

struct timeval tv_timeo = { 0, 500000 };   // 0.5 seconds

struct struct_t_UDP
{
    int    Socket;
    struct sockaddr_in ServerAddr;
    //double   TXBuffer[DF_UDP_BUFFER_SIZE];
    double   RXBuffer[DF_UDP_BUFFER_SIZE];
};

#pragma pack(push,1)
struct RX_message_data
{
    double data_to_send_1;
    double data_to_send_2;
    double data_to_send_3;

};
#pragma pack(pop)

struct struct_t_UDP           StrUDP;
struct sockaddr_in            MyAddr;
struct RX_message_data        RX_buff;

// node main loop, for ROS
int main(int argc, char** argv)
{
    // node name initialization
    init(argc, argv, "mbzirc_ugv_udp_client");


    // assign node handler
    ros::NodeHandle nh_;

    ros::Rate loop_rate(10);

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




    ros::Publisher example_pub;
    std_msgs::Float64MultiArray example_array;
    // example_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/received_from_udp",100);
    // example_array.data.at(0) = received_1;
    // example_array.data.at(1) = received_2;
    // example_array.data.at(2) = received_3;
    // example_pub.publish(example_array);

    int temp_data;

    while(ros::ok())
    {
        memset(StrUDP.RXBuffer, 0, sizeof(StrUDP.RXBuffer));
        temp_data = recvfrom(StrUDP.Socket, StrUDP.RXBuffer, sizeof(StrUDP.RXBuffer), 0, (struct sockaddr *)(&StrUDP.ServerAddr), (socklen_t *)&size_addr);

        memcpy(&RX_buff, (RX_message_data*)StrUDP.RXBuffer, sizeof(RX_message_data));

        double received_1 = RX_buff.data_to_send_1;
        double received_2 = RX_buff.data_to_send_2;
        double received_3 = RX_buff.data_to_send_3;

        ROS_INFO("recv data : %f  %f  %f \n",RX_buff.data_to_send_1,RX_buff.data_to_send_2, RX_buff.data_to_send_3);

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


