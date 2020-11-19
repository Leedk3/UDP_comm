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

#include <GeographicLib/UTMUPS.hpp>
#include "projector/UTM.h"

// setup the initial name
using namespace ros;
using namespace std;
using namespace usrg_utm;

#define DF_UDP_BUFFER_SIZE_  256
#define DF_UDP_PORTNUM_      16098
#define DF_UDP_SERVER_ADDR_  "192.168.10.201" 

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

        double m_origin_lat;
        double m_origin_lon;
        

};

KAIST_TO_ETRI::KAIST_TO_ETRI(ros::NodeHandle& n) : nh(n), bVehState(false), bVehOdometry(false), bGpsRaw(false), 
                                                   bImuRaw(false), bCollision(false)
{

    //Parameters
    nh.param<double>("/ugv_odom_lanelet2/origin_lat", m_origin_lat, 36.6104614); //36.48378670 : sejong APT
    nh.param<double>("/ugv_odom_lanelet2/origin_lon", m_origin_lon, 127.2891284); //127.294427 : sejong

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
    // Use UTM
    sensor_msgs::NavSatFix origin_llh;
    origin_llh.latitude =  m_origin_lat;
    origin_llh.longitude = m_origin_lon;

    geometry_msgs::Pose2D current_odom;
    current_odom.
    UtmProjector projector(origin_llh);    
    geometry_msgs::Pose2D projection = projector.forward(*msg);


}

void KAIST_TO_ETRI::CallbackGpsRaw(const sensor_msgs::NavSatFix& msg)
{
    m_GpsRaw = msg;
    bGpsRaw = true;

    sensor_msgs::NavSatFix origin_llh;
    origin_llh.latitude = 36.613100;
    origin_llh.longitude = 127.4697269;
    
    UtmProjector projector(origin_llh);    
    geometry_msgs::Pose2D projection = projector.forward(msg);


    sensor_msgs::NavSatFix testBackward = projector.reverse(projection);

    printf("utm local Data: \n x: %.9f ,y: %.9f \n" , projection.x, projection.y);
    printf("origin llh Data: \n x: %.9f ,y: %.9f \n" , testBackward.latitude, testBackward.longitude);

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
    double gpsHeading;
    uint32_t gpsStatus;
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
    ServerAddr.sin_port        = htons(DF_UDP_PORTNUM_); // PORT#
    ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR_); // IP for Server (Normally PC IP)
    

    KAIST_TO_ETRI _server_to_send(nh_);

    while(ros::ok()){

        TX_buff.vehLat = (double)_server_to_send.m_VehOdometry.pose.pose.position.x; 
        TX_buff.vehLng = (double)_server_to_send.m_VehOdometry.pose.pose.position.y; 
        TX_buff.vehHeading = (double)_server_to_send.m_VehOdometry.pose.pose.orientation.w; 
        // Odom to llh


        TX_buff.XAcc = (double)_server_to_send.m_ImuRaw.linear_acceleration.x; 
        TX_buff.YAcc = (double)_server_to_send.m_ImuRaw.linear_acceleration.y; 
        TX_buff.ZAcc = (double)_server_to_send.m_ImuRaw.linear_acceleration.z; 
        TX_buff.XGyro = (double)_server_to_send.m_ImuRaw.angular_velocity.x; 
        TX_buff.YGyro = (double)_server_to_send.m_ImuRaw.angular_velocity.y; 
        TX_buff.ZGyro = (double)_server_to_send.m_ImuRaw.angular_velocity.z; 
        TX_buff.gpsTime = (double)_server_to_send.m_GpsRaw.header.stamp.toSec(); 
        TX_buff.gpsLat = (double)_server_to_send.m_GpsRaw.latitude; 
        TX_buff.gpsLng = (double)_server_to_send.m_GpsRaw.longitude; 
        TX_buff.gpsHeight = (double)_server_to_send.m_GpsRaw.altitude;
        TX_buff.gpsHeading =  (double)15.0;
        TX_buff.gpsStatus = (uint32_t)_server_to_send.m_GpsRaw.status.status; //ublox data output: -1. chech this. 
        TX_buff.TTC = (float)0; //float //Calculate using front obstacle.
        TX_buff.collision = (bool)_server_to_send.m_Collision.data; 
        TX_buff.crossTrackErr = (double)0; //Calculate using stanley method.
        TX_buff.headingErr = (double)0; //calculate using stanley method.
        TX_buff.vehStuck = (float)0; 
        TX_buff.operationMode = (uint8_t)0; 
        TX_buff.RTOKAIST = (uint16_t)0; //Error , Status 
        TX_buff.steeringAngle = (double)0; //remove it
        TX_buff.frontObstacleDetection = (double)0; //What is this?
        TX_buff.rearObstacleDetection =  (double)0; //What is this?

        //comm test
        // TX_buff.vehLat = (double)1.0;
        // TX_buff.vehLng =  (double)2.0;
        // TX_buff.vehHeading = (double)3.0; 
        // TX_buff.XAcc = (double)4.0;
        // TX_buff.YAcc =  (double)5.0;
        // TX_buff.ZAcc =  (double)6.0;
        // TX_buff.XGyro =  (double)7.0;
        // TX_buff.YGyro =  (double)8.0;
        // TX_buff.ZGyro =  (double)9.0;
        // TX_buff.gpsTime =  (double)11.0;
        // TX_buff.gpsLat =  (double)12.0;
        // TX_buff.gpsLng =  (double)13.0;
        // TX_buff.gpsHeight =  (double)14.0;
        // TX_buff.gpsHeading =  (double)15.0;
        // TX_buff.gpsStatus = (uint32_t)5;
        // TX_buff.TTC =  (float)16.0;
        // TX_buff.collision = (bool)true; 
        // TX_buff.crossTrackErr = (double)18.0; 
        // TX_buff.headingErr = (double)19.0;
        // TX_buff.vehStuck = (float)20.0;
        // TX_buff.operationMode = (uint8_t)3;  
        // TX_buff.RTOKAIST = (uint16_t)4;
        // TX_buff.steeringAngle = (double)21.0;  
        // TX_buff.frontObstacleDetection = (double)22.0;  
        // TX_buff.rearObstacleDetection =  (double)23.0;


        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));
        // std::cout << "--------------------------------------------------------" << std::endl;
        // ROS_INFO("TX data to Etri -  CrossTrackErr: %f, HeadingErr : %f)", TX_buff.crossTrackErr, TX_buff.headingErr);
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