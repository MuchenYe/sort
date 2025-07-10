#include "wheeltec_robot.h"
// #include "Quaternion_Solution.h"


/**************************************
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheeltec_robot"); 
  turn_on_robot Robot_Control;  //实例化一个对象
  Robot_Control.Control();      //循环执行数据采集和发布话题等操作
  ROS_INFO("wheeltec_robot node has turned on ");  //提示已创建wheeltec_robot节点 
  return 0;  
} 
/**************************************
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  short  transition;  //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //帧头0X7B
  Send_Data.tx[1] = 0; //预留位
  Send_Data.tx[2] = 0; //预留位

  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux.linear.x*1000; //将浮点数放大一千倍，简化传输
  std::cout<<"__linear.x:"<< transition <<std::endl;
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux.linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;
  //ROS_INFO("x:%lf\n"%twist_aux.linear.x)

  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux.angular.z*1000;
  std::cout<<"__angular.z:"<< transition <<std::endl;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);  //BBC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL;  //帧尾0X7D
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof (Send_Data.tx)); //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
  }
}
/**************************************
功能: 发布sensor相关信息
***************************************/
void turn_on_robot::Publish_Sensor_Count()
{
    std_msgs::UInt8 sensorCnt_msgs; 
    sensorCnt_msgs.data = sensor_count; 
    sensorCnt_publisher.publish(sensorCnt_msgs);
}
/**************************************
功能: 发布pitch
***************************************/
void turn_on_robot::Publish_Pitch()
{
    std_msgs::Int16 pitch_msgs; 
    pitch_msgs.data = pitch; 
    pitch_publisher.publish(pitch_msgs);
}
/**************************************
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BBC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0)  //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
     check_sum=check_sum^Receive_Data.rx[k]; //按位异或
  }
  if(mode==1)//发送数据模式
  {
   for(k=0;k<Count_Number;k++)
     check_sum=check_sum^Send_Data.tx[k]; //按位异或  
  }
  return check_sum; //返回按位异或结果
}
/**************************************
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{ 
  short transition_16=0, j=0, Header_Pos=0, Tail_Pos=0; //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0}; //临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));  //通过串口读取下位机发送过来的数据

  //直接查看接收到的原始数据，调试使用
  /*
  ROS_INFO("%x-%x",Receive_Data_Pr[0],Receive_Data_Pr[1])
  */ 

  //记录帧头帧尾位置
  for(j=0; j<RECEIVE_DATA_SIZE; j++)
  {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
       Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
       Tail_Pos=j;    
  }

  if(Tail_Pos==(Header_Pos + RECEIVE_DATA_SIZE - 1))
    //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));

  else if(Header_Pos==(1+Tail_Pos))
  {
    //如果帧头在帧尾后面，纠正数据位置后复制数据包到Receive_Data.rx
    for(j=0;j<RECEIVE_DATA_SIZE;j++)
        Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%RECEIVE_DATA_SIZE];
  }
  else 
    return false;
      
  /* //查看Receive_Data.rx，调试使用
  ROS_INFO("%x-%x",Receive_Data.rx[0],Receive_Data.rx[1]) */

  Receive_Data.Frame_Header= Receive_Data.rx[0]; //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail= Receive_Data.rx[RECEIVE_DATA_SIZE-1];  //数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER ) //判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //判断帧尾
    { 
      if (Receive_Data.rx[RECEIVE_DATA_SIZE-2] == Check_Sum(RECEIVE_DATA_SIZE-2, READ_DATA_CHECK)||(Header_Pos==(1+Tail_Pos)))
      {
        Receive_Data.Flag_Stop=Receive_Data.rx[1];  //预留位
        sensor_count = Receive_Data.rx[2]; //获取sensor count
        
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[3]<<8;
        transition_16 |=  Receive_Data.rx[4]; 
        pitch = transition_16; //获取pitch
        
        return true;
     }
    }
  } 
  return false;
}
/**************************************
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  while(ros::ok())
  {
    if (true == Get_Sensor_Data())//通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      Publish_Sensor_Count();  //发布电源电压话题
      Publish_Pitch();     //发布pitch话题
    }
   
    ros::spinOnce();   //循环等待回调函数
    }
}
/**************************************
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot():sensor_count(0),pitch(0)
{
  //清空数据
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));

  ros::NodeHandle private_nh("~"); //创建节点句柄
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  // private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/wheeltec_controller");//固定串口号
  private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyACM0");//固定串口号
  private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200); //和下位机通信波特率115200
  private_nh.param<std::string>("cmd_vel",          cmd_vel,          "cmd_vel"); //速度控制指令

  sensorCnt_publisher = n.advertise<std_msgs::UInt8>("SensorCount", 10);  //创建电池电压话题发布者
  pitch_publisher   = n.advertise<std_msgs::Int16>("Pitch", 20);         //创建pitch话题发布者

  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub     = n.subscribe(cmd_vel, 50 , &turn_on_robot::Cmd_Vel_Callback, this); 

  ROS_INFO_STREAM("Data ready"); //提示信息
  
  try
  { 
    Stm32_Serial.setPort(usart_port_name); //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); 
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("can not open serial port,Please check the serial port cable! ");  //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM(" serial port opened");  //串口开启成功提示
  }
}
/**************************************
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port");//如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close();  //关闭串口  
  ROS_INFO_STREAM("Shutting down"); //提示信息
}




