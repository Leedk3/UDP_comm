#define UAV_NUM1

// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using standard messages, float 64 type
// communicate to image processing algorithm result
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>
#include <sensor_msgs/NavSatFix.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

void VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg);
float steering_angle;
float speed;

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      10699
#define DF_UDP_SERVER_ADDR  "192.168.10.201"
//#define DF_UDP_SERVER_ADDR  "127.0.0.1"

double llh[3];

#pragma pack(1)
/*struct struct_t_UDP
{
    int    Socket;
    struct sockaddr_in ServerAddr;
    //double  TXBuffer[DF_UDP_BUFFER_SIZE];
    //char   RXBuffer[DF_UDP_BUFFER_SIZE];
};*/

struct TX_message_data
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t veh_mode;
    uint8_t brake_cmd;
    int8_t  steer_cmd;
    uint8_t powertrain_mode;
    uint8_t speed_cmd;
    uint8_t chk_sum_1;
    uint8_t chk_sum_2;
};

#pragma pack()

// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "UnmannedSol_UDP_TX");

    // assign node handler
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Server_Avante_LLH_TX\n");

    //Publish
    //Publisher  Avante_LLH_pub	= nh_.advertise<std_msgs::Float64MultiArray>("/Avante/LLH", 10);

    ros::Subscriber subVehCommand = nh_.subscribe("/Ackermann/command",10,&VehCommandCallback);

    ros::Rate loop_rate(5);

    int    Socket;
    struct sockaddr_in ServerAddr;
    //struct struct_t_UDP           StrUDP;
    //struct sockaddr_in            MyAddr;
    struct TX_message_data        TX_buff;


    // Socket Creation
    Socket = socket(PF_INET, SOCK_DGRAM, 0);
    int enable = 1;
    setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    if(Socket == -1){
        printf("[ERROR] 'socket()'\n");
        return -1;
    }
    else{
        printf("[DONE] UDP socket is created\n");
    }

    // UDP-IP Setting
    memset(&ServerAddr, 0, sizeof(ServerAddr)); // Clear to 0
    ServerAddr.sin_family      = PF_INET;
    ServerAddr.sin_port        = htons(DF_UDP_PORTNUM); // PORT#
    ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR); // IP for Server (Normally PC IP)

    while(ok()){
        printf("Server_Avante!\n");

        TX_buff.header_1 = 80;
        TX_buff.header_2 = 85;
        TX_buff.veh_mode = 15;
        TX_buff.brake_cmd = 0;
        TX_buff.steer_cmd = 50;
        TX_buff.powertrain_mode = 0;
        TX_buff.speed_cmd = 10;
        TX_buff.chk_sum_1 = 1;
        TX_buff.chk_sum_2 = 2;

        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));

        // loop rate [Hz]
        loop_rate.sleep();

        // loop sampling, ros
        spinOnce();
    }
    // for debugging
    printf("Terminate: Server_Avante_LLH_TX\n");
    close(Socket);

    return 0;
}

void VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    steering_angle = msg.steering_angle;
    speed = msg.speed;
}
