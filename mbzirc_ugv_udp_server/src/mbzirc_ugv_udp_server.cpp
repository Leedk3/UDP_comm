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

//void ubloxCallback(const sensor_msgs::NavSatFix& ublox_msg);

// setup the initial name
using namespace ros;
using namespace std;

//ros::Subscriber UBLOX_sub;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      3764
#define DF_UDP_CLINET_ADDR  "192.168.3.51" //"127.0.0.1"
// #define DF_UDP_CLINET_ADDR  "127.0.0.1"

class SERVER_TO_SEND
{
    public:
        SERVER_TO_SEND(ros::NodeHandle& n);        
        ~SERVER_TO_SEND();
        void example_callback(const std_msgs::Float64ConstPtr& msg);
        
        double data_to_send_1;
        double data_to_send_2;
        double data_to_send_3;

    private:        
        ros::NodeHandle nh;
        ros::Subscriber example_sub;
        
};

SERVER_TO_SEND::SERVER_TO_SEND(ros::NodeHandle& n) 
{
    example_sub = nh.subscribe("/exmaple_topic", 10, &SERVER_TO_SEND::example_callback, this);
    ROS_DEBUG("SERVER_TO_SEND is created");
};

SERVER_TO_SEND::~SERVER_TO_SEND() 
{    
    ROS_INFO("SERVER_TO_SEND destructor.");
}

void SERVER_TO_SEND::example_callback(const std_msgs::Float64ConstPtr& msg){
    data_to_send_1 = msg->data;
    data_to_send_2 = data_to_send_1 * 2;
    data_to_send_3 = data_to_send_1 * 3; 
}

#pragma pack(1)

struct TX_message_data
{
    double send_via_udp_1;
    double send_via_udp_2;
    double send_via_udp_3;
};
#pragma pack()


// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "mbzirc_ugv_udp_server");

    // assign node SERVER_TO_SEND
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Server_Avante_LLH_TX\n");
    ros::Rate loop_rate(10);

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
    ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_CLINET_ADDR); // IP for Server (Normally PC IP)
    

    SERVER_TO_SEND _server_to_send(nh_);

    while(ros::ok()){
        TX_buff.send_via_udp_1 = _server_to_send.data_to_send_1;
        TX_buff.send_via_udp_2 = _server_to_send.data_to_send_2;
        TX_buff.send_via_udp_3 = _server_to_send.data_to_send_3;	

        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));
        ROS_INFO("We sended (%f, %f, %f)", TX_buff.send_via_udp_1,TX_buff.send_via_udp_2,TX_buff.send_via_udp_3);
        loop_rate.sleep();
        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Server_Avante_LLH_TX\n");
    close(Socket);

    return 0;
}

