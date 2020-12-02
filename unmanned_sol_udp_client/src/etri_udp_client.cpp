#include <ros/ros.h>
// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      16101
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
    uint8_t operationMode;
    uint8_t emergencyStop;
    double desPoseX;
    double desPoseY;
    double desTheta;
    double desSteer;
    double desSpeed;
    double temp1;
    double temp2;
};
#pragma pack(pop)


class ETRI_TO_KAIST
{
    public:
        ETRI_TO_KAIST(ros::NodeHandle& n);        
        ~ETRI_TO_KAIST();
        ros::NodeHandle nh;
        
};

ETRI_TO_KAIST::ETRI_TO_KAIST(ros::NodeHandle& n) 
{
    ROS_DEBUG("ETRI_TO_KAIST is created");
};

ETRI_TO_KAIST::~ETRI_TO_KAIST() 
{    
    ROS_INFO("ETRI_TO_KAIST destructor.");
}


// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "UnmannedSol_UDP_RX");

    // assign node ETRI_TO_KAIST
    ros::NodeHandle nh_;
    ros::Publisher pubEtriOperator = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/Ackermann/command/etri", 10);
    ros::Publisher pubEtriJoy = nh_.advertise<sensor_msgs::Joy>("/joy/etri_mobile", 10);

    ackermann_msgs::AckermannDriveStamped EtriCommand;
    sensor_msgs::Joy EtriMobile;
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
    
    ETRI_TO_KAIST _client_to_receive(nh_);
    int temp_data;
    while(ros::ok())
    {
        memset(StrUDP.RXBuffer, 0, sizeof(StrUDP.RXBuffer));
        temp_data = recvfrom(StrUDP.Socket, StrUDP.RXBuffer, sizeof(StrUDP.RXBuffer), 0, (struct sockaddr *)(&StrUDP.ServerAddr), (socklen_t *)&size_addr);

        memcpy(&RX_buff, (RX_message_data*)StrUDP.RXBuffer, sizeof(RX_message_data));

        uint8_t operationMode = RX_buff.operationMode;
        uint8_t emergencyStop = RX_buff.emergencyStop;
        double desPoseX = RX_buff.desPoseX;
        double desPoseY = RX_buff.desPoseY;
        double desTheta = RX_buff.desTheta;
        double desSteer = RX_buff.desSteer;
        double desSpeed = RX_buff.desSpeed;
        double temp1 = RX_buff.temp1;
        double temp2 = RX_buff.temp2;

        std::cout << "---------------------------"<< std::endl;
        std::cout << "operationMode: " << operationMode << std::endl;
        std::cout << "emergencyStop: " << emergencyStop << std::endl;
        std::cout << "desPoseX: " << desPoseX << std::endl;
        std::cout << "desPoseY: " << desPoseY << std::endl;
        std::cout << "desTheta: " << desTheta << std::endl;
        std::cout << "desSteer: " << desSteer << std::endl;
        std::cout << "desSpeed: " << desSpeed << std::endl;
        std::cout << "temp1: " << temp1 << std::endl;
        std::cout << "temp2: " << temp2 << std::endl;
        std::cout << "\n" << std::endl;

        EtriCommand.header.stamp = ros::Time::now();
        EtriCommand.header.frame_id = "base_link";
        EtriCommand.drive.steering_angle = desSteer;
        EtriCommand.drive.speed = desSpeed;
        pubEtriOperator.publish(EtriCommand);
        
        switch(operationMode) // mode chage
        {
            case 1: //Maunal
                EtriMobile.buttons.at(5) = 1; //R1 button : manual
                EtriMobile.buttons.at(3) = 0; //rectangle botton : visual
                EtriMobile.buttons.at(1) = 1; //circle botton :auto
                EtriMobile.buttons.at(0) = 0; //x button : waiting mode
                break;
            case 2: //Visual
                EtriMobile.buttons.at(5) = 0; //R1 button : manual
                EtriMobile.buttons.at(3) = 1; //rectangle botton : visual
                EtriMobile.buttons.at(1) = 0; //circle botton :auto
                EtriMobile.buttons.at(0) = 0; //x button : waiting mode
                break;
            case 3: //Autonomous
                EtriMobile.buttons.at(5) = 0; //R1 button : manual
                EtriMobile.buttons.at(3) = 0; //rectangle botton : visual
                EtriMobile.buttons.at(1) = 1; //circle botton :auto
                EtriMobile.buttons.at(0) = 0; //x button : waiting mode            
                break;
            default:
                EtriMobile.buttons.at(0) = 0; //R1 button : manual
                EtriMobile.buttons.at(0) = 0; //rectangle botton : visual
                EtriMobile.buttons.at(0) = 0; //circle botton :auto
                EtriMobile.buttons.at(0) = 1; //x button : waiting mode            
                break;
        }

        if(emergencyStop == 1)
        {
            EtriMobile.buttons.at(0) = 0; //R1 button : manual
            EtriMobile.buttons.at(0) = 0; //rectangle botton : visual
            EtriMobile.buttons.at(0) = 0; //circle botton :auto
            EtriMobile.buttons.at(0) = 1; //x button : waiting mode            
        }
        if(temp1 == 1)
            EtriMobile.buttons.at(4) = 1; //camera init
        else
        {
            EtriMobile.buttons.at(4) = 0; //camera init
        }
        
        pubEtriJoy.publish(EtriMobile);


        loop_rate.sleep();
        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Client\n");
    close(StrUDP.Socket);

    return 0;
}

