#ifndef KAIST_TO_ETRI_
#define KAIST_TO_ETRI_
// essential header for ROS-OpenCV operation
#include <ros/ros.h>
// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      16099 //Check here
#define DF_UDP_SERVER_ADDR  "192.168.10.201" 

class KAIST_TO_ETRI
{
    public:
        KAIST_TO_ETRI(ros::NodeHandle& n);        
        ~KAIST_TO_ETRI();

        void CallbackVehState(const ackermann_msgs::AckermannDriveStamped& msg);        
        void CallbackOdometry(const nav_msgs::Odometry& msg);
        void CallbackGpsRaw(const sensor_msgs::NavSatFix& msg);
        void CallbackImuRaw(const sensor_msgs::Imu& msg);
        void CallbackCollision(const std_msgs::Bool& msg);
        
        ackermann_msgs::AckermannDriveStamped m_VehState;
        nav_msgs::Odometry m_VehOdometry;
        sensor_msgs::NavSatFix m_GpsRaw;
        sensor_msgs::Imu m_ImuRaw;
        std_msgs::Bool m_Collision;

        bool bVehState;
        bool bVehOdometry;
        bool bGpsRaw;
        bool bImuRaw;
        bool bCollision;
        
        ros::NodeHandle nh;
        
        ros::Subscriber subVehState;
        ros::Subscriber subVehOdometry;
        ros::Subscriber subGpsRaw;
        ros::Subscriber subImuRaw;
        ros::Subscriber subCollision;
        
};

KAIST_TO_ETRI::KAIST_TO_ETRI(ros::NodeHandle& n) : bVehState(false), bVehOdometry(false), bGpsRaw(false), 
                                                   bImuRaw(false), bCollision(false)
{
    subVehState = nh.subscribe("/Ackeramnn/veh_state",10,&KAIST_TO_ETRI::CallbackVehState, this);
    subVehOdometry = nh.subscribe("/Odometry/ekf_slam",10,&KAIST_TO_ETRI::CallbackOdometry, this);
    subGpsRaw = nh.subscribe("/gps/fix",10, &KAIST_TO_ETRI::CallbackGpsRaw, this);
    subImuRaw = nh.subscribe("/imu/data",10, &KAIST_TO_ETRI::CallbackImuRaw, this);
    subCollision = nh.subscribe("/Bool/Collision",10, &KAIST_TO_ETRI::CallbackCollision, this);

    ROS_DEBUG("KAIST_TO_ETRI is created");
};

KAIST_TO_ETRI::~KAIST_TO_ETRI() 
{    
    ROS_INFO("KAIST_TO_ETRI destructor.");
}

void KAIST_TO_ETRI::CallbackVehState(const ackermann_msgs::AckermannDriveStamped& msg)
{
    m_VehState = msg;
    bVehState = true;
}

void KAIST_TO_ETRI::CallbackOdometry(const nav_msgs::Odometry& msg)
{
    m_VehOdometry = msg;
    bVehOdometry = true;
}

void KAIST_TO_ETRI::CallbackGpsRaw(const sensor_msgs::NavSatFix& msg)
{
    m_GpsRaw = msg;
    bGpsRaw = true;
}

void KAIST_TO_ETRI::CallbackImuRaw(const sensor_msgs::Imu& msg)
{
    m_ImuRaw = msg;
    bImuRaw = true;
}
void KAIST_TO_ETRI::CallbackCollision(const std_msgs::Bool& msg)
{   
    m_Collision = msg;
    bCollision = true;
}

#pragma pack(1)
struct TX_message_data
{
    double vehLat;
    double vehLng;
    double vehHeading;
    double XAcc;
    double YAcc;
    double ZAcc;
    double XGyro;
    double YGyro;
    double ZGyro;
    double gpsTime;
    double gpsLat;
    double gpsLng;
    double gpsHeight;
    double gpsStatus;
    float TTC;
    bool collision;
    double crossTrackErr;
    double headingErr;
    float vehStuck;
    uint8_t operationMode;
    uint16_t RTOKAIST;
    double steeringAngle;
    double frontObstacleDetection;
    double rearObstacleDetection; 
};
#pragma pack()


// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "Kaist2Etri_UDP_TX");

    // assign node KAIST_TO_ETRI
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Kaist2Etri Server_TX\n");
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
    

    KAIST_TO_ETRI _server_to_send(nh_);

    while(ros::ok()){

        TX_buff.vehLat = _server_to_send.m_VehOdometry.pose.pose.position.x; 
        TX_buff.vehLng = _server_to_send.m_VehOdometry.pose.pose.position.y; 
        TX_buff.vehHeading = _server_to_send.m_VehOdometry.pose.pose.orientation.w; 
        TX_buff.XAcc = _server_to_send.m_ImuRaw.linear_acceleration.x; 
        TX_buff.YAcc = _server_to_send.m_ImuRaw.linear_acceleration.y; 
        TX_buff.ZAcc = _server_to_send.m_ImuRaw.linear_acceleration.z; 
        TX_buff.XGyro = _server_to_send.m_ImuRaw.angular_velocity.x; 
        TX_buff.YGyro = _server_to_send.m_ImuRaw.angular_velocity.y; 
        TX_buff.ZGyro = _server_to_send.m_ImuRaw.angular_velocity.z; 
        TX_buff.gpsTime = _server_to_send.m_GpsRaw.header.stamp.toSec(); 
        TX_buff.gpsLat = _server_to_send.m_GpsRaw.latitude; 
        TX_buff.gpsLng = _server_to_send.m_GpsRaw.longitude; 
        TX_buff.gpsHeight = _server_to_send.m_GpsRaw.altitude; 
        TX_buff.gpsStatus = (double)_server_to_send.m_GpsRaw.status.status; //change this to int8 
        TX_buff.TTC = 0; //float 
        TX_buff.collision = _server_to_send.m_Collision.data; 
        TX_buff.crossTrackErr = 0; 
        TX_buff.headingErr = 0; 
        TX_buff.vehStuck = 0; 
        TX_buff.operationMode = 0; 
        TX_buff.RTOKAIST = 0; 
        TX_buff.steeringAngle = 0; 
        TX_buff.frontObstacleDetection = 0; 
        TX_buff.rearObstacleDetection = 0; 

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

#endif //KAIST_TO_ETRI