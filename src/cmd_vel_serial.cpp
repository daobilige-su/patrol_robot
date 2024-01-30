//// Created by zzs on 24-1-9.

#include "ros/ros.h"
#include <boost/asio.hpp>
#include "geometry_msgs/Twist.h"
#include "yaml-cpp/yaml.h"
#include <ros/package.h>

int getCrc16(unsigned char *bufData, unsigned int buflen, unsigned char *pcrc);

////Get param(wheel diameter and dist) from the yaml
std::string package_path = ros::package::getPath("patrol_robot");
std::string yaml_path=package_path+"/param/ptcld_to_scan.yaml";
YAML::Node config_yaml=YAML::LoadFile(yaml_path);//调用yaml文件，（需要apt-install:libyaml-cpp-dev并修改camke添加库文件）
const double wheel_diameter=config_yaml["wheel_diameter"].as<double>(),wheel_dist=config_yaml["wheel_dist"].as<double>();
//const double wheel_diameter=300;
//const double wheel_dist=630;

//Set Serial_port
boost::asio::io_service iosev;
boost::asio::serial_port g_serial_port(iosev, "/dev/ttyUSB0");//robot_chassis
boost::system::error_code err;
const unsigned char HEADER[2] = {0x55, 0x02};
const unsigned char END[2] = {0x0d, 0x0a};
//const unsigned char HEADER_RECV[2] = {0xaa, 0x01};
//const unsigned char HEADER_IMU = 0x03;

union SendData
{
    short value;
    unsigned char data[2];
} g_leftVelSend, g_rightVelSend,liu_f,liu_r;



/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子速度，发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void sendVelCmd(double linear_vel_x, double angular_vel_z)
{
    ROS_INFO(" +++++++++++ sendVelCmd +++++++++++ \n");
    ROS_INFO("linear_x  %f",linear_vel_x);
    ROS_INFO("angular_x  %f",angular_vel_z);
    linear_vel_x=linear_vel_x*1000; //m/s to mm/s
    unsigned char buf[8] = {0}; //
    int i = 0;
    //Calculate respectively velocity of left and right wheel
    double lv=linear_vel_x-angular_vel_z*(wheel_dist/2.0);
    double rv=linear_vel_x+angular_vel_z*(wheel_dist/2.0);
    g_leftVelSend.value=short(lv);
    g_rightVelSend.value=short(rv);
    ROS_INFO("Left vel to SEND: %d ", g_leftVelSend.value);
    ROS_INFO("Right vel to SEND: %d ", g_rightVelSend.value);

//    // 计算左右轮期望速度(转弯半径算法)
//    double r = linear_vel_x / angular_vel_z; // mm,Radius of rotation
//    if (linear_vel_x == 0) // 旋转
//    {
//        ROS_INFO("Rotation");
//        liu_f.value = (short)(-angular_vel_z * wheel_dist/2.0); // mm/s
//        liu_r.value = (short)(angular_vel_z * wheel_dist/2.0); // mm/s
//    }
//    else if (angular_vel_z<0.001 && angular_vel_z>-0.001) // 直线
//    {
//        ROS_INFO("Straight");
//        liu_f.value = (short)linear_vel_x; // mm/s
//        liu_r.value = (short)linear_vel_x;
//        //ROS_INFO("Left vel to SEND: %d ", g_leftVelSend.value);
//    }
//    else // 转弯
//    {
//        ROS_INFO("Turn");
//        liu_f.value = (short)(angular_vel_z * (r - wheel_dist/2.0)); // mm/s
//        liu_r.value = (short)(angular_vel_z * (r + wheel_dist/2.0));
//    }
//    ROS_INFO("Left vel to Liu: %d ", liu_f.value);
//    ROS_INFO("Right vel to Liu: %d ", liu_r.value);


    // 设置消息头
    for (i = 0; i < 2; i++)
        buf[i] = HEADER[i];
    // 设置机器人左右轮速度
    for (i = 0; i < 2; i++)
    {
        buf[i + 2] = g_leftVelSend.data[i];
        buf[i + 4] = g_rightVelSend.data[i];
    }
    buf[6]=END[0];
    buf[7]=END[1];
    // 预留控制指令
    //int length = 4;
    //buf[2 + length] = ctrlFlag;
    // 设置校验值、消息尾
    //getCrc16(buf, 3 + length, buf + 3 + length);
    // 通过串口下发数据
    for(i=0;i<8;i++){
    	std::cout<<std::uppercase << std::hex << std::setfill('0') << std::setw(2)<<(buf[i]&0xff)<<" "<<std::endl;
    }
    boost::asio::write(g_serial_port, boost::asio::buffer(buf));
    ROS_INFO("--------------- sendVelCmd -------------- \n");
}


void Callback(const geometry_msgs::Twist::ConstPtr& msg){
    double vx = msg->linear.x;
    double w = msg->angular.z;
    sendVelCmd(vx, w);
}
//    double lv = vx-w*(wheel_dist/2.0); // left wheel's velocity
//    double rv = vx+w*(wheel_dist/2.0);// right wheel's velocity
////     set motor
////     print('lv: ' .. lv .. ', rv: ' .. rv) -- for debugging purpose
//    double lv_rot = (lv/(wheel_diameter/2.0)); // rotation velocity of left wheel in radian/s, no need to *(180/math.pi)
//    double rv_rot = (rv/(wheel_diameter/2.0)); // rotation velocity of right wheel in radian/s, no need to *(180/math.pi)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_serial_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("cmd_vel", 2, Callback); //
    ros::spin();  //ros::spin()库是响应循环，消息到达时调用函数chatterCallback，CTRL+C结束循环
    return 0;
}

//check by crc
int getCrc16(unsigned char *bufData, unsigned int buflen, unsigned char *pcrc)
{
    int ret = 0;
    unsigned short CRC = 0xffff;
    unsigned short POLYNOMIAL = 0xa001;
    int i, j;

    // 指令为空
    if (bufData == NULL || pcrc == NULL)
    {
        return -1;
    }

    // 校验计算的长度为0
    if (buflen == 0)
    {
        return ret;
    }

    for (i = 0; i < buflen; i++)
    {
        CRC ^= bufData[i];

        // 总共八次右移操作
        for (j = 0; j < 8; j++)
        {
            if ((CRC & 0x0001) != 0)
            {
                // 右移的移出位为1
                CRC >>= 1;
                CRC ^= POLYNOMIAL;
            }
            else
            {
                // 右移的移出位为0
                CRC >>= 1;
            }
        }
    }

    // printf("CRC=%X\n", CRC);
    // 低位在前，高位在后
    pcrc[0] = (unsigned char)(CRC & 0x00ff);
    pcrc[1] = (unsigned char)(CRC >> 8);

    return ret;
}
