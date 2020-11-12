#define UAV_NUM1
// essential header for ROS-OpenCV operation
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

#define STEERING_RESOLUTION_GAIN 5 // resolution: 0.2 = 1 / STEERING_RESOLUTION_GAIN
#define SPEED_RESOLUTION_GAIN 10 // resolution: 0.1 = 1 / SPEED_RESOLUTION_GAIN
 


class SERVER_TO_SEND
{
    public:
        SERVER_TO_SEND(ros::NodeHandle& n);        
        ~SERVER_TO_SEND();
        void VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg);
        float steering_angle;
        float speed;
        
        ros::NodeHandle nh;
        ros::Subscriber subVehCommand;
        
};

SERVER_TO_SEND::SERVER_TO_SEND(ros::NodeHandle& n) 
{
    subVehCommand = nh.subscribe("/Ackermann/command/joy",10,&SERVER_TO_SEND::VehCommandCallback, this);
    // example_sub = nh.subscribe("/exmaple_topic", 10, &SERVER_TO_SEND::example_callback, this);
    ROS_DEBUG("SERVER_TO_SEND is created");
};

SERVER_TO_SEND::~SERVER_TO_SEND() 
{    
    ROS_INFO("SERVER_TO_SEND destructor.");
}

void SERVER_TO_SEND::VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    steering_angle = msg.drive.steering_angle;
    if (msg.drive.speed >= 0)
        speed = msg.drive.speed;
    else{
        speed = 0;
    }
}

#pragma pack(1)

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

    // assign node SERVER_TO_SEND
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Server_TX\n");
    ros::Rate loop_rate(25);

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
    

    SERVER_TO_SEND _server_to_send(nh_);

    while(ros::ok()){

        // int8_t steerToSend = steering_angle * STEERING_RESOLUTION_GAIN 
        TX_buff.header_1 = 80;
        TX_buff.header_2 = 85;
        TX_buff.veh_mode = 15;
        TX_buff.brake_cmd = 0;
        TX_buff.steer_cmd = _server_to_send.steering_angle * STEERING_RESOLUTION_GAIN;
        TX_buff.powertrain_mode = 0;
        TX_buff.speed_cmd = _server_to_send.speed * SPEED_RESOLUTION_GAIN;
        TX_buff.chk_sum_1 = 0;
        TX_buff.chk_sum_2 = 0;
        
        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));
        // std::cout << "--------------------------------------------------------" << std::endl;
        // ROS_INFO("Steering cmd:  %f, Speed cmd : %f)", _server_to_send.steering_angle, _server_to_send.speed);
        // ROS_INFO("TX data to Unmanned: STEER: %d, SPEED : %d)", TX_buff.steer_cmd, TX_buff.speed_cmd);

        loop_rate.sleep();
        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Server_TX\n");
    close(Socket);

    return 0;
}

