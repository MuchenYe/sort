#include "ros/ros.h"
#include "serial/serial.h"
// #include "serial_pkg/speed.h"
#include "chrono"
#include<iostream>
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "serial_pkg/road_image.h"
#include "serial_pkg/red.h"
#include "std_msgs/Float32.h"

#define Start_Byte 0xA5
#define End_Byte 0x5A
#define Servo_INIT 1400
// 2023.05.27 21:37  调试 29s 跑完全场。 主要部分霍夫变换+判断一边线+中线两边扫

serial::Serial ser;
// �������ݻ�����
unsigned char send_buff[9];
// opencv捕捉
cv::Mat frame;
// cv::VideoCapture cap;
cv::VideoCapture cap(0,cv::CAP_V4L2);


road_ns::road_image road_image; // 命名空间 ：：类  对象
red_ns::red red;
int sidewalk_ = 0;

int8_t state = 0;    // ��¼״̬
int8_t lightnum = 0; // ��¼���̵�

// init_all() 内的标志位，此处有5个
bool sidewalk = false;
bool ramp = false;
bool rate_limiting_on = false;
bool rate_limiting_off = false;
bool left_turn = false;
// init_all() 内的标志位

int limit = 0;
int leftWait = 0;
int leftturn = 0;
int greenwait = 0;
unsigned char signageLast;
unsigned char signage;
static bool left_turn_flag = false;
static bool show_image = false;
static bool limit_time_flag = true;
static bool light_time_flag = true;
static bool right_time_flag = true;
static bool stop_time_flag = true;
static bool left_state_flag = false;

typedef union
{
    float float_data;
    unsigned char byte_data[4];
} Float_Byte;

struct
{
    Float_Byte vx;
    Float_Byte vx_last;
    uint16_t turn_pwm;
    uint16_t last_turn_pwm;
} speed_structure;

/**lin
 * @brief 串口发送函数
 * @attention   send_buff[9]   :  0 开始位 0xA5 | 1-4 速度  | 5-6 转弯pwm  | 7 校验 |  8   结束位0x5A
 */
void Serial_Send(float vx, uint16_t turn_pwm)
{
    speed_structure.vx.float_data = vx;
    speed_structure.turn_pwm = turn_pwm;
    // speed_structure.vx.float_data *= 1.1;
    // speed_structure.turn_pwm = 700;

    send_buff[1] = speed_structure.vx.byte_data[0];
    send_buff[2] = speed_structure.vx.byte_data[1];
    send_buff[3] = speed_structure.vx.byte_data[2];
    send_buff[4] = speed_structure.vx.byte_data[3];

    send_buff[5] = ((speed_structure.turn_pwm & 0xFF00) >> 8);
    send_buff[6] = speed_structure.turn_pwm & 0x00FF;

    send_buff[7] = send_buff[1] ^ send_buff[2] ^ send_buff[3] ^ send_buff[4] ^ send_buff[5] ^ send_buff[6]; // 林：crc校验位
    ser.write(send_buff, 9);

    ROS_INFO("VX:%.2f  TURN:%d", speed_structure.vx.float_data, speed_structure.turn_pwm);
}

/*
    @brief
    林：串口初始化
*/
int serial_init(void)
{
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 1000ms
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized");
    else
        return -1;

    // 串口发送数组初始化
    send_buff[0] = Start_Byte;
    send_buff[8] = End_Byte;
    speed_structure.vx.float_data = 0;
    speed_structure.turn_pwm = 700;

    return 0;
}
//打开摄像头函数
// int open_video(void)
// {
// //增加cv::CAP_V4L2，即可解决
//         //增加cv::CAP_V4L2，即可解决

//     cap.open(0, cv::CAP_V4L2);  //0-笔记本自带摄像头，1-外接usb双目摄像头
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter.fourcc('Y', 'U', 'Y', 'V'))
//     cap.set(cv::CAP_PROP_FORMAT, cv::CV_8UC2)
//     cap.set(cv::CAP_PROP_MODE, ccv::CAP_MODE_YUYV)

//     // cap.set(cv::CAP_PROP_FRAME_WIDTH, 6);  //设置捕获视频的宽度
//     // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 4);  //设置捕获视频的高度
//     // cap.set(cv::CAP_PROP_FPS, 5);
//     if (!cap.isOpened())
//     {
//         ROS_ERROR("ERROR! Unable to open camera");
//         return -1;
//     }

//     return 0;

// }
int open_video(void)
{

    int device_ID = 0;
    int api_ID = cv::CAP_ANY;

    cap.open(device_ID , api_ID);
    //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    if (!cap.isOpened())
    {
        ROS_ERROR("ERROR! Unable to open camera");
        return -1;
    }

    return 0;
}

/*ALL初始化函数*/
int init_all(int argc)
{
    if (serial_init() == -1)
        return -1;
    if (open_video() == -1)
        return -1;

    // 初始化ros里的参数，我暂时认为无所谓
    ros::param::set("sidewalk", false);
    ros::param::set("ramp", false);
    ros::param::set("rate_limiting_on", false);
    ros::param::set("rate_limiting_off", false);
    ros::param::set("left_turn", false);
    ros::param::set("road_state", 0);

    // 图像对象的标志位，先从人行横道开始 ，值为0
    road_image.road_state = road_image.SideWalk;
    // 不知道拿来做什么
    if (argc > 1)
        show_image = true;

    return 0;
}

/*个人认为此函数很奇怪，先注释掉*/

// void turn(uint16_t *pwm, uint16_t left, uint16_t right)
// {
//     if (*pwm == 800)
//     {
//         *pwm = left;
//     }
//     if (*pwm == 2000)
//     {
//         *pwm = right;
//     }
// }

//此转向函数林修改了，在大于一定值后进行左右值选择，被选择量均是通过调试经验取的
void turn(uint16_t *pwm, uint16_t left, uint16_t right)
{
    if (*pwm <= 800)
    {
        *pwm = left;
    }
    if (*pwm >= 2000)
    {
        *pwm = right;
    }
}


/*i think that this is used to see the state of road*/
void switch_state(int param)
{
    // road_state不等于红绿灯时，能右转；反之不行
    if (param != 5)
        right_time_flag = true;

    // 判断状态
    switch (param)
    {
    case 0:
        road_image.road_state = road_image.SideWalk;
        break;
    case 1:
        road_image.road_state = road_image.Ramp;
        break;
    case 2:
        road_image.road_state = road_image.Limit_Rate_Ready;
        break;
    case 3:
        road_image.road_state = road_image.Limit_Rate_On;
        break;
    case 4:
        road_image.road_state = road_image.S_Turn;
        break;
    case 5:
        road_image.road_state = road_image.Light;
        break;
    case 6:
        road_image.road_state = road_image.Left_Turn;
        break;
    case 7:
        road_image.road_state = road_image.Stop;
        break;
    default:
        break;
    }
}
// check 参数服务器内的参数，然后丢到switch_state()里给road_state进行赋值
// void signage_check(float vx, uint16_t turn_pwm)
// {
//     // 从参数服务器中获取init_all（）设置 的参数
//     ros::param::get("sidewalk", sidewalk);
//     ros::param::get("ramp", ramp);
//     ros::param::get("rate_limiting_on", rate_limiting_on);
//     ros::param::get("rate_limiting_off", rate_limiting_off);
//     ros::param::get("left_turn", left_turn);
//     int local_state = road_image.road_state;
//     ros::param::get("road_state", local_state);
//     switch_state(local_state);

//     static double limit_time = ros::Time::now().toSec();
//     static double light_time = ros::Time::now().toSec();
//     static double right_time = ros::Time::now().toSec();
//     static double stop_time = ros::Time::now().toSec();

//     switch (road_image.road_state)
//     {
//     case road_image.SideWalk:
//         vx =0.5;                  // 1.25 1.2
        //turn(&turn_pwm, 578, 700); // 565 555 565 //560
        // if (turn_pwm == 578)
        //     vx = 1.15; // 1.1
        // else
        //     turn_pwm = 700;
        // if(turn_pwm >= 740)
        //     turn_pwm = 740;
        // if (turn_pwm == 578)
        // sidewalk_++;
        // if (sidewalk && sidewalk_ >= 5)
        // {
        //     std::cout << "________sidewalk_________" << std::endl;
        //     road_image.road_state = road_image.Ramp;
        //     ros::param::set("road_state", road_image.road_state);
        //     Serial_Send(0.0, 1400);      // 1400
        //     ros::Duration(1.1).sleep(); // 停车1.1s
        // }
        // break;
    // case road_image.Ramp:
    //     sidewalk_ = 0;
    //     turn(&turn_pwm, 555, 710);
    //     vx = 1.15; // 1.1 1.05 1.0 0.95 0.9 0.85  0.8
    //     if (ramp)
    //     {
    //         road_image.road_state = road_image.Limit_Rate_Ready;
    //         ros::param::set("road_state", road_image.road_state);
    //         Serial_Send(0.0, turn_pwm);
    //         ros::Duration(1.1).sleep();
    //     }
    //     break;
    // case road_image.Limit_Rate_Ready:
    //     vx = 1.0; // 0.85 0.8
    //     turn(&turn_pwm, 540, 700);
    //     if (turn_pwm == 540)
    //         vx = 1.2;
    //     if (rate_limiting_on)
    //     {
    //         road_image.road_state = road_image.Limit_Rate_On;
    //         ros::param::set("road_state", road_image.road_state);
    //         // Serial_Send(vx,760);
    //         // ros::Duration(0.2).sleep(); // 0.2
    //     }
    //     break;
    // case road_image.Limit_Rate_On:
    //     if (limit_time_flag)
    //     {
    //         limit_time = ros::Time::now().toSec();
    //         limit_time_flag = false;
    //     }
    //     turn(&turn_pwm, 700, 700);
    //     vx = 0.5;
    //     if (rate_limiting_off || (ros::Time::now().toSec() - limit_time > 5.0))
    //     {
    //         road_image.road_state = road_image.S_Turn;
    //         ros::param::set("road_state", road_image.road_state);
    //         limit_time_flag = true;
    //     }
    //     break;
    // case road_image.S_Turn:
    //     vx = 1.0;                  // 1.0 1.1  1.0 0.95 0.9
    //     turn(&turn_pwm, 568, 800); // 565 565 563 565 580
    //     if (turn_pwm == 568 && left_state_flag == false)
    //     {
    //         left_state_flag = true;
    //     }
    //     if (turn_pwm != 568 && left_state_flag == true)
    //     {
    //         vx = 0.35; // 0.55 0.6 0.51 0.55
    //         if (turn_pwm == 800)
    //         {
    //             road_image.road_state = road_image.Light;
    //             ros::param::set("road_state", road_image.road_state);
    //             left_state_flag = false;
    //         }
    //         // turn_pwm = 800;
    //     }
    //     break;
    // case road_image.Light:
    //     if (right_time_flag)
    //     {
    //         right_time = ros::Time::now().toSec();
    //         right_time_flag = false;
    //     }
    //     vx = 1.0;                  // 0.95 0.85 0.82 0.9 0.82
    //     turn(&turn_pwm, 700, 838); // old_right : 827 831 831 825
    //     if (turn_pwm < 660)
    //         turn_pwm = 660;
    //     if (turn_pwm > 740 && turn_pwm < 838) // 831
    //         turn_pwm = 740;
    //     if (turn_pwm == 838)
    //     {
    //         float temp_time = ros::Time::now().toSec() - right_time;
    //         // ROS_ERROR("%.2f",temp_time);
    //         if (temp_time < 3.0)
    //             vx = 1.1; // 1.2
    //         else if (temp_time < 4.9)
    //             vx = 1.5;
    //         else
    //             vx = 0.6;
    //     }
    //     if (left_turn)
    //     {
    //         Serial_Send(0.0, 700); // turn_pwm
    //         // ros::Duration(3).sleep();
    //         bool light_detect = false;
    //         if (light_time_flag)
    //         {
    //             light_time = ros::Time::now().toSec();
    //             light_time_flag = false;
    //         }
    //         while (light_detect == false) // light_detect == false
    //         {
    //             cap.read(frame);
    //             light_detect = red.Red_Judge(frame);
    //             if (ros::Time::now().toSec() - light_time > 10.0)
    //                 break;
    //         }
    //         road_image.road_state = road_image.Left_Turn;
    //         ros::param::set("road_state", road_image.road_state);
    //         light_time_flag = true;
    //     }
    //     break;
    // case road_image.Left_Turn:
    //     vx = 1.15; // 1.2
    //                // if(stop_time_flag)
    //                // {
    //                //     stop_time = ros::Time::now().toSec();
    //                //     stop_time_flag = false;
    //                // }
    //     turn(&turn_pwm, 540, 720);
    //     if (turn_pwm == 540)
    //         vx = 0.85; // 0.8 0.75
    //     // if(ros::Time::now().toSec() - stop_time > 4.0)
    //     // {
    //     //     road_image.road_state = road_image.Stop;
    //     //     ros::param::set("road_state",road_image.road_state);
    //     //     stop_time_flag = true;
    //     // }
    //     break;
    // case road_image.Stop:
    //     vx = 0.0;
    //     turn_pwm = 700;
    //     break;
    // default:
    //     break;
//     }
//     Serial_Send(vx, turn_pwm);
// }
// // check 参数服务器内的参数，然后丢到switch_state()里给road_state进行赋值
void signage_check(float vx, uint16_t turn_pwm)
{
    // 从参数服务器中获取init_all（）设置 的参数
    ros::param::get("sidewalk", sidewalk);
    ros::param::get("ramp", ramp);
    ros::param::get("rate_limiting_on", rate_limiting_on);
    ros::param::get("rate_limiting_off", rate_limiting_off);
    ros::param::get("left_turn", left_turn);
    int local_state = road_image.road_state;
    ros::param::get("road_state", local_state);
    switch_state(local_state);

    static double limit_time = ros::Time::now().toSec();
    static double light_time = ros::Time::now().toSec();
    static double right_time = ros::Time::now().toSec();
    static double stop_time = ros::Time::now().toSec();

    switch (road_image.road_state)
    {
        //从开始到左弯一直标志位给人行横道
    case road_image.SideWalk:
        vx = 0.6;//1.2;//1.5;                  // 1.25 1.2   //开始道左弯的速度
        turn(&turn_pwm, 1100, 1450); // 565 555 565 //560
        // if (turn_pwm <= 1300)
        //     sidewalk_++;
        if (sidewalk)// && sidewalk_ >=3)
        {
            ROS_INFO("________sidewalk_________");	
            road_image.road_state = road_image.Ramp;
            ros::param::set("road_state", road_image.road_state);
            Serial_Send(0.0, turn_pwm);      //1400  ， 700
            ros::Duration(1.1).sleep(); // 停车1.1s
        }

        break;
    case road_image.Ramp:
        sidewalk_ = 0;
        turn(&turn_pwm, 1300, 1500);
        vx = 0.6;//1.15; // 1.1 1.05 1.0 0.95 0.9 0.85  0.8
        if (ramp)
        {
            ROS_INFO("________ramp_________");
            road_image.road_state = road_image.Limit_Rate_Ready;
            ros::param::set("road_state", road_image.road_state);
            Serial_Send(0.0, turn_pwm);
            ros::Duration(1.1).sleep();
        }
        break;
    case road_image.Limit_Rate_Ready:
        vx = 0.3;//1.0; // 0.85 0.8
        turn(&turn_pwm, 1100, 1450);
        if (turn_pwm <= 1200)
            vx = 0.3;
        if (rate_limiting_on)
        {
            ROS_INFO("________limit_on_________");
            road_image.road_state = road_image.Limit_Rate_On;
            ros::param::set("road_state", road_image.road_state);
            // Serial_Send(vx,760);
            // ros::Duration(0.2).sleep(); // 0.2
        }
        break;
    case road_image.Limit_Rate_On:
        if (limit_time_flag)
        {
            limit_time = ros::Time::now().toSec();
            limit_time_flag = false;
        }
        //turn(&turn_pwm, 1100, 1500);
        vx = 0.25;
        if (rate_limiting_off)// || (ros::Time::now().toSec() - limit_time > 5.0))
        {
            ROS_INFO("________limit_off_________");
            road_image.road_state = road_image.S_Turn;
            ros::param::set("road_state", road_image.road_state);
            limit_time_flag = true;
        }
        break;
    case road_image.S_Turn:
        vx = 0.5;                  // 1.0 1.1  1.0 0.95 0.9
       turn(&turn_pwm, 1100, 1300); // 565 565 563 565 580
        if (turn_pwm <= 1300 && left_state_flag == false)
        {
            ROS_INFO("________S_turn_________");
            left_state_flag = true;
        }
       // if (turn_pwm != 1100 && left_state_flag == true)
        if (left_state_flag == true)
        {
            vx = 0.3; // 0.55 0.6 0.51 0.55
            if (turn_pwm >= 1500)
            {
                road_image.road_state = road_image.Light;
                ros::param::set("road_state", road_image.road_state);
                left_state_flag = false;
            }
            // turn_pwm = 800;
        }
        break;
    case road_image.Light:
        if (right_time_flag)//右转的时间标志符号
        {
            ROS_INFO("________lin light_________");
            right_time = ros::Time::now().toSec();  //记录当前时间
            right_time_flag = false;
        }
        vx = 0.3;//1.0;                  // 0.95 0.85 0.82 0.9 0.82
        //turn(&turn_pwm, 1100, 1600); // old_right : 827 831 831 825
       // if (turn_pwm > 740 && turn_pwm < 838) // 831
        //    turn_pwm = 740;
        if (turn_pwm == 1600)
        {
            float temp_time = ros::Time::now().toSec() - right_time;
            // ROS_ERROR("%.2f",temp_time);
            if (temp_time < 3.0)
                vx = 0.3; // 1.2
            else if (temp_time < 4.9)
                vx = 0.3;//1.5;
            else
                vx = 0.3;
        }
       turn(&turn_pwm,1400,1700);//add
        if (left_turn)
        {
            Serial_Send(0.0, 1400); // turn_pwm
            ros::Duration(3).sleep();
            bool light_detect = false;
            if (light_time_flag)       //灯记录时间符号
            {
                light_time = ros::Time::now().toSec();
                light_time_flag = false;
            }
            while (light_detect == false) // light_detect == false
            {
                cap.read(frame);
                light_detect = red.Red_Judge(frame);  //绿灯判断成功退出
                if (ros::Time::now().toSec() - light_time > 10.0)  //等待时间过长也退出
                    break;
            }
            road_image.road_state = road_image.Left_Turn;  //左转符号
            ros::param::set("road_state", road_image.road_state);
            light_time_flag = true;
        }
        break;
    case road_image.Left_Turn:
        vx = 0.5;//1.15; // 1.2
                   // if(stop_time_flag)
                   // {
                   //     stop_time = ros::Time::now().toSec();
                   //     stop_time_flag = false;
                   // }
        turn(&turn_pwm, 1200, 1450);
        if (turn_pwm <= 1250)
            vx = 0.3; // 0.8 0.75
        // if(ros::Time::now().toSec() - stop_time > 4.0)
        // {
        //     road_image.road_state = road_image.Stop;
        //     ros::param::set("road_state",road_image.road_state);
        //     stop_time_flag = true;
        // }
        break;
    case road_image.Stop:
        vx = 0.0;
        turn_pwm = 1400;
        break;
    default:
        break;
    }
    Serial_Send(vx, turn_pwm);
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "serial_send_msg");
//     ros::NodeHandle nh;
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise("camera/road_image", 5);

//     road_image.road_init();

//     if (init_all(argc) == -1)
//         return -1;

 
//     ros::Rate loop_rate(30);

//     //cv::VideoCapture cap(0, cv::CAP_V4L2);
//     //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
//     // int fourcc = cap.get(cv::CAP_PROP_FOURCC);
//     // if (fourcc == cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))
//     // {
//     //     std::cout << "视频捕获格式已成功设置为 MJPG" << std::endl;
//     // }
//     // else
//     // {
//     //     std::cout << "视频捕获格式设置失败" << std::endl;
//     // }

//         // 设置视频分辨率
// 	// int width = 640;
// 	// int height = 480;
// 	// cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
// 	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

//     //double setFrameRate;
    
//     // double frameRate = 30.0;
// 	// cap.set(cv::CAP_PROP_FPS, frameRate);
//     // road_image.vx = 0.5;
//     // ros::Rate loop_rate(100);
//     while (ros::ok())
//     {
//         // 图像
//        cap.read(frame);
//         // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//         // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
//         // cv::namedWindow("result", 0);
// 		// cv::imshow("frame", frame);
// 		// cv::waitKey(1);
//     // //    double rate=cap.get(cv::CAP_PROP_FPS);
// 	// //    std::cout << "The frame rate is: " << rate <<std::endl;
//        if(!frame.empty())
//         {    
//             sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//             pub.publish(msg);
//     //     // setFrameRate = cap.get(cv::CAP_PROP_FPS);
//     //     // std::cout << "帧率:" <<setFrameRate<< std::endl;
//         //处理
//         road_image.Calc_Speed(&frame, show_image);
//         signage_check(road_image.vx, road_image.turn_pwm);
//     //     // setFrameRate = cap.get(cv::CAP_PROP_FPS);
//     //     // std::cout << "帧率:" <<setFrameRate<< std::endl;
//     //     //循环
//     //     // Serial_Send(0.5, 1400);
//         }

//         loop_rate.sleep();
//     }
//     //Serial_Send(0.0, 1400);
//     return 0;
// }

int main( int argc, char** argv )
{
   ros::init(argc, argv, "serial_send_msg");
    //打开摄像头
    // cv::VideoCapture capture(0);    //使用 ls /dev/vid*查看使用的摄像头
    // cv::VideoCapture capture(0);    //使用 ls /dev/vid*查看使用的摄像头
    // capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // capture.open(0);
     cv::VideoCapture capture(0);
    //cv::VideoCapture capture("../overpass.mp4");
    //int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    //capture.set(cv::CAP_PROP_FOURCC, fourcc);
    if(!capture.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    //创建显示窗口
    // if(!capture.isOpened()){
    //     std::cout << "Error opening video stream or file" << std::endl;
    //     return -1;
    // }
    cv::namedWindow("USB Camera", cv::WINDOW_AUTOSIZE);
    
 
    //逐帧显示
    while(true)
    {
        cv::Mat img;
        capture >> img;
        if(img.empty())
        {
            std::cout << "Fail to read image from camera!" << std::endl;
            break;
        }
        if (img.empty()) continue;
        // int new_width,new_height,width,height,channel;
        // width=img.cols;
        // height=img.rows;
        // channel=img.channels();
        // std::cout<<width<<"  "<<height<<"  "<<channel<<std::endl;
 
        // new_width=800;
        // if(width>800)
        // {
        //     new_height=int(new_width*1.0/width*height);
        // }
 
        // cv::resize(img, img, cv::Size(new_width, new_height));
        imshow("USB Camera",img);
 
        int keycode = cv::waitKey(30) & 0xff ; //ESC键退出
            if (keycode == 27) break ;
    }
 
    capture.release();
    cv::destroyAllWindows() ;
}


// int main( int argc, char** argv )
// {
//   std::cout << cv::getBuildInformation() << std::endl;  
// }