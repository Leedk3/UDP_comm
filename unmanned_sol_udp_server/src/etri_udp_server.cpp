#ifndef KAIST_TO_ETRI_
#define KAIST_TO_ETRI_
// essential header for ROS-OpenCV operation
#include <ros/ros.h>
#include <cmath>
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
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

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

        void init();

        void CallbackVehState(const ackermann_msgs::AckermannDriveStamped& msg);        
        void CallbackOdometry(const nav_msgs::Odometry& msg);
        void CallbackGpsRaw(const sensor_msgs::NavSatFix& msg);
        void CallbackImuRaw(const sensor_msgs::Imu& msg);
        void CallbackCollision(const std_msgs::Bool& msg);
        void CallbackOperationMode(const std_msgs::Int32& msg);
        void CallbackTargetObject(const visualization_msgs::Marker& msg);
        
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
        ros::Subscriber subOperationMode;
        ros::Subscriber subTargetObject;

        double m_origin_lat;
        double m_origin_lon;
        double m_utm2gps_yaw_bias;
        sensor_msgs::NavSatFix m_origin_llh;
        sensor_msgs::NavSatFix m_estimated_llh;
        double m_heading;
        int m_OperationMode;
        int m_RTO2KAIST;
        double m_TTC;
        double m_CountStuck;
        double m_currentTime;
        double m_prevTime;
        geometry_msgs::Pose2D m_projectionPrev;
        double m_GpsYawRaw;


};

KAIST_TO_ETRI::KAIST_TO_ETRI(ros::NodeHandle& n) : nh(n), bVehState(false), bVehOdometry(false), bGpsRaw(false), 
                                                   bImuRaw(false), bCollision(false)
{

    //Parameters
    nh.param<double>("/etri_udp_server/origin_lat", m_origin_lat, 37.6970297); //36.48378670 : sejong APT
    nh.param<double>("/etri_udp_server/origin_lon", m_origin_lon, 126.7495693); //127.294427 : sejong
    nh.param<double>("/etri_udp_server/utm2gps_yaw_bias", m_utm2gps_yaw_bias, -20); //deg

    subVehState = nh.subscribe("/Ackermann/veh_state",10,&KAIST_TO_ETRI::CallbackVehState, this);
    subVehOdometry = nh.subscribe("/Odometry/ekf_slam",10,&KAIST_TO_ETRI::CallbackOdometry, this);
    subGpsRaw = nh.subscribe("/gps/fix",10, &KAIST_TO_ETRI::CallbackGpsRaw, this);
    subImuRaw = nh.subscribe("/imu/data",10, &KAIST_TO_ETRI::CallbackImuRaw, this);
    subCollision = nh.subscribe("/Bool/collision",10, &KAIST_TO_ETRI::CallbackCollision, this);
    subOperationMode = nh.subscribe("/Int/OperationMode",10, &KAIST_TO_ETRI::CallbackOperationMode, this);
    subTargetObject = nh.subscribe("/Marker/VisualServoing/TargetObject", 10, &KAIST_TO_ETRI::CallbackTargetObject, this);

    ROS_DEBUG("KAIST_TO_ETRI is created");
    init();
};

KAIST_TO_ETRI::~KAIST_TO_ETRI() 
{    
    ROS_INFO("KAIST_TO_ETRI destructor.");
}

void KAIST_TO_ETRI::init()
{
    
    m_origin_llh.latitude =  m_origin_lat;
    m_origin_llh.longitude = m_origin_lon;

}

void KAIST_TO_ETRI::CallbackVehState(const ackermann_msgs::AckermannDriveStamped& msg)
{
    m_VehState = msg;
    bVehState = true;
}

void KAIST_TO_ETRI::CallbackOperationMode(const std_msgs::Int32& msg)
{
    m_OperationMode = msg.data;
    if(msg.data == 4)
        m_RTO2KAIST = 0;
    else if(msg.data > 0 && msg.data < 4)
        m_RTO2KAIST = 1;
    else
    {
        m_RTO2KAIST = 2;
        ROS_ERROR("Operation Mode ERROR");
    }
    
}

void KAIST_TO_ETRI::CallbackOdometry(const nav_msgs::Odometry& msg)
{
    m_VehOdometry = msg;
    bVehOdometry = true;
    
    // Use UTM
    geometry_msgs::Pose2D current_odom;

    double yaw_bias = m_utm2gps_yaw_bias * M_PI / 180; //deg -> rad

    current_odom.x = -msg.pose.pose.position.y;
    current_odom.y = msg.pose.pose.position.x;

    current_odom.x = current_odom.x * cos(yaw_bias) + current_odom.y * sin(yaw_bias);
    current_odom.y = -current_odom.x * sin(yaw_bias) + current_odom.y * cos(yaw_bias);

    UtmProjector utm2gps(m_origin_llh);    
    m_estimated_llh = utm2gps.reverse(current_odom);

    tf::Quaternion q(msg.pose.pose.orientation.x, 
                     msg.pose.pose.orientation.y, 
                     msg.pose.pose.orientation.z, 
                     msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, m_heading);
    m_heading *=  180 / M_PI;
    ROS_INFO("lat: %f, lon: %f, heading: %f", m_estimated_llh.latitude, m_estimated_llh.longitude, m_heading);

}

void KAIST_TO_ETRI::CallbackGpsRaw(const sensor_msgs::NavSatFix& msg)
{
    m_GpsRaw = msg;
    bGpsRaw = true;
    
    UtmProjector gps2utm(m_origin_llh);    
    geometry_msgs::Pose2D projection = gps2utm.forward(msg);

    double roll, pitch;
    m_GpsYawRaw = atan2(projection.y - m_projectionPrev.y , projection.x - m_projectionPrev.x);
    m_projectionPrev = projection;

}

void KAIST_TO_ETRI::CallbackImuRaw(const sensor_msgs::Imu& msg)
{
    m_ImuRaw = msg;
    bImuRaw = true;
}
void KAIST_TO_ETRI::CallbackCollision(const std_msgs::Bool& msg)
{   
    m_currentTime = ros::Time::now().toSec();
    m_Collision = msg;

    double dt = m_currentTime - m_prevTime;
    if(!bCollision)
    {
        m_prevTime = m_currentTime;    
        bCollision = true;
        return;
    }
    
    if(msg.data)
        m_CountStuck += dt;
    else
    {
        m_CountStuck = 0;
    }

    m_prevTime = m_currentTime;
}

void KAIST_TO_ETRI::CallbackTargetObject(const visualization_msgs::Marker& msg)
{

    double distToTargetObject = sqrt(pow(msg.pose.position.x,2) + 
                                     pow(msg.pose.position.y,2));
    if(distToTargetObject == 0)
        m_TTC = 10;
    m_TTC =  distToTargetObject / (m_VehState.drive.speed * 3.6) ;

    if(m_TTC > 10)
        m_TTC = 10.1;
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

        TX_buff.vehLat = (double)_server_to_send.m_estimated_llh.latitude; 
        TX_buff.vehLng = (double)_server_to_send.m_estimated_llh.longitude; 
        TX_buff.vehHeading = (double)_server_to_send.m_heading;         // Odom to llh

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
        TX_buff.gpsHeading =  (double)_server_to_send.m_GpsYawRaw;
        TX_buff.gpsStatus = (uint32_t)1; //ublox data output: -1. chech this. 
        TX_buff.TTC = (float)_server_to_send.m_TTC; 
        TX_buff.collision = (bool)_server_to_send.m_Collision.data; 
        TX_buff.crossTrackErr = (double)0; //Calculate using stanley method. /ssh
        TX_buff.headingErr = (double)0; //calculate using stanley method. /ssh
        TX_buff.vehStuck = (float)_server_to_send.m_CountStuck; //ssh
        TX_buff.operationMode = (uint8_t)_server_to_send.m_OperationMode; 
        TX_buff.RTOKAIST = (uint16_t)_server_to_send.m_RTO2KAIST; //Error , Status 
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