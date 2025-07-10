#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Imu.h>
using namespace std;

//宏定义
#define SEND_DATA_CHECK   1          //发送数据校验标志位
#define READ_DATA_CHECK   0          //接收数据校验标志位
#define FRAME_HEADER      0XA5       //帧头
#define FRAME_TAIL        0X5A       //帧尾
#define RECEIVE_DATA_SIZE 7          //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //ROS向下位机发送的数据的长度
#define PI 		  3.1415926f         //PI 

//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;	       
	float Y_speed;           
	float Z_speed;         
	unsigned char Frame_Tail; 
}SEND_DATA;

//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
        unsigned char Frame_Header;
	uint8_t sensor_count;  
	int16_t pitch; 	
	unsigned char Frame_Tail;
}RECEIVE_DATA;

//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
	public:
		turn_on_robot();  
		~turn_on_robot(); 
		void Control();    //循环控制代码
		serial::Serial Stm32_Serial; //声明串口对象 
	private:
		ros::NodeHandle n;           //创建ROS节点句柄

		ros::Subscriber Cmd_Vel_Sub; //初始化话题订阅者
		//速度话题订阅回调函数，其它小车使用此函数声明
		void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);              

		ros::Publisher sensorCnt_publisher, pitch_publisher; //初始化话题发布者
		void Publish_Sensor_Count();    //发布sensor count话题
		void Publish_Pitch();           //发布pitch话题

        bool Get_Sensor_Data();    //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC校验函数

        string usart_port_name, cmd_vel; //定义相关变量
        int serial_baud_rate;       //串口通信波特率
        RECEIVE_DATA Receive_Data;  //串口接收数据结构体
        SEND_DATA Send_Data;        //串口发送数据结构体

        uint8_t sensor_count;       //电源电压
        int16_t pitch;              // pitch
};
#endif

