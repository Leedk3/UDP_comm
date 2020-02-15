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

void ubloxCallback(const sensor_msgs::NavSatFix& ublox_msg);

// setup the initial name
using namespace ros;
using namespace std;

ros::Subscriber UBLOX_sub;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      3764
#define DF_UDP_SERVER_ADDR  "192.168.3.111"
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
    double lat;
    double lon;
    double height;
};
#pragma pack()

// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "udp_server");

    // assign node handler
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Server_Avante_LLH_TX\n");

    //Publish
    //Publisher  Avante_LLH_pub	= nh_.advertise<std_msgs::Float64MultiArray>("/Avante/LLH", 10);

    ros::Subscriber UBLOX_sub = nh_.subscribe("/ublox_gps/fix",10,&ubloxCallback);

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
    //ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //if(bind(Socket,(struct sockaddr *)&ServerAddr, sizeof(sockaddr_in)) < 0){
    //    printf("bind() error!\n");
    //    return -1;
    //}

//	memset(&MyAddr, 0, sizeof(MyAddr));
//	MyAddr.sin_family = PF_INET;
//	MyAddr.sin_addr.s_addr = htonl(INADDR_ANY);
//	MyAddr.sin_port = htons(DF_UDP_PORTNUM);

//	if(bind(StrUDP.Socket,(struct sockaddr *)&MyAddr, sizeof(MyAddr))!=0)
//	{
//		printf("bind() error!\n");
//        return -1;
//	}

//    char recvBuff;

//	int size_addr = sizeof(StrUDP.ServerAddr);

    //int ros_count = 0;
    //float lat; float lon; float height;

    // node loop, for ROS, check ros status, ros::ok()
    while(ok()){
        printf("Server_Avante!\n");

        TX_buff.lat	 = llh[0];					// Avante
        TX_buff.lon	 = llh[1];					// Avante
        TX_buff.height   = 69;		// Avante

        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));
        printf("send data : %.7f  %.7f  %.7f\n",TX_buff.lat, TX_buff.lon, TX_buff.height);

        // messages
        //std_msgs::Float64MultiArray Avante_LLH_msg;

        //Avante_LLH_msg.data.clear();
        //Avante_LLH_msg.data.resize(4);
        //Avante_LLH_msg.data[0] = TX_buff.lat;
        //Avante_LLH_msg.data[1] = TX_buff.lon;
        //Avante_LLH_msg.data[2] = TX_buff.height;
        //Avante_LLH_msg.data[2] = 69;
        //Avante_LLH_pub.publish(Avante_LLH_msg);

        //ros_count++;

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

void ubloxCallback(const sensor_msgs::NavSatFix& ublox_msg)
{
        llh[0] = ublox_msg.latitude;		//Lat
        llh[1] = ublox_msg.longitude;		//Lon
        llh[2] = ublox_msg.altitude;		//Height
        return;
}

/*void ubloxCallback(const std_msgs::Float64MultiArray& ublox_msg)
{
        llh[0] = ublox_msg.data.at(3)/100.0;		//Lat
        llh[1] = ublox_msg.data.at(5)/100.0;		//Lon
        llh[2] = 69;		//vehicle class (emergency vehicle)
        return;
}*/

