#include <ros/ros.h>
// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      16099
#define DF_UDP_SERVER_ADDR  "192.168.10.201"
#define DF_UDP_CLIENT_ADDR  "192.168.10.204"	//"127.0.0.1" -> local 

#define STEERING_RESOLUTION_GAIN 5 // resolution: 0.2 = 1 / STEERING_RESOLUTION_GAIN
#define SPEED_RESOLUTION_GAIN 10 // resolution: 0.1 = 1 / SPEED_RESOLUTION_GAIN


struct timeval tv_timeo = { 0, 500000 };   // 0.5 seconds

struct struct_t_UDP
{
    int    Socket;
    struct sockaddr_in ServerAddr;
    double   RXBuffer[DF_UDP_BUFFER_SIZE];
};
#pragma pack(push,1)
struct RX_message_data
{
    uint8_t header_1;
    uint8_t header_2;
    int8_t steering_angle;
    int8_t speed;
    uint8_t chk_sum_1;
    uint8_t chk_sum_2;


};
#pragma pack(pop)


class CLIENT_TO_RECEIVE
{
    public:
        CLIENT_TO_RECEIVE(ros::NodeHandle& n);        
        ~CLIENT_TO_RECEIVE();
        // void VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg);
        // float steering_angle;
        // float speed;
        
        ros::NodeHandle nh;
        
};

CLIENT_TO_RECEIVE::CLIENT_TO_RECEIVE(ros::NodeHandle& n) 
{
    // subVehCommand = nh.subscribe("/Ackermann/command",10,&CLIENT_TO_RECEIVE::VehCommandCallback, this);
    // example_sub = nh.subscribe("/exmaple_topic", 10, &CLIENT_TO_RECEIVE::example_callback, this);
    ROS_DEBUG("CLIENT_TO_RECEIVE is created");
};

CLIENT_TO_RECEIVE::~CLIENT_TO_RECEIVE() 
{    
    ROS_INFO("CLIENT_TO_RECEIVE destructor.");
}


// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "UnmannedSol_UDP_RX");

    // assign node CLIENT_TO_RECEIVE
    ros::NodeHandle nh_;
    ros::Publisher pubAckermann = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/Ackermann/veh_state", 10);
    ackermann_msgs::AckermannDriveStamped VehState;
    // for debugging
    printf("Initiate: Server_RX\n");
    ros::Rate loop_rate(50);

    int    Socket;

    struct struct_t_UDP           StrUDP;
    struct sockaddr_in            MyAddr;
    struct RX_message_data        RX_buff;


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
    MyAddr.sin_addr.s_addr = inet_addr(DF_UDP_CLIENT_ADDR);

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
    
    CLIENT_TO_RECEIVE _client_to_receive(nh_);
    int temp_data;
    while(ros::ok())
    {
        memset(StrUDP.RXBuffer, 0, sizeof(StrUDP.RXBuffer));
        temp_data = recvfrom(StrUDP.Socket, StrUDP.RXBuffer, sizeof(StrUDP.RXBuffer), 0, (struct sockaddr *)(&StrUDP.ServerAddr), (socklen_t *)&size_addr);

        memcpy(&RX_buff, (RX_message_data*)StrUDP.RXBuffer, sizeof(RX_message_data));

        uint8_t header_1 = RX_buff.header_1;
        uint8_t header_2 = RX_buff.header_2;
        int8_t steering_angle = RX_buff.steering_angle;
        int8_t speed = RX_buff.speed;
        uint8_t chk_sum_1 = RX_buff.chk_sum_1;
        uint8_t chk_sum_2 = RX_buff.chk_sum_2;
        

        ROS_INFO("recv data : %d , %d  \n",RX_buff.steering_angle,RX_buff.speed);

        VehState.header.frame_id = "base_link";
        VehState.header.stamp = ros::Time::now();
        VehState.drive.steering_angle = (float)steering_angle / STEERING_RESOLUTION_GAIN; //Unit: Degree
        VehState.drive.speed = (float)speed / SPEED_RESOLUTION_GAIN; //Unit: 
        pubAckermann.publish(VehState);

        loop_rate.sleep();
        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Client\n");
    close(StrUDP.Socket);

    return 0;
}

