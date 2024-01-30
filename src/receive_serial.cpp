// Created by zzs on 24-1-9.
//
#include "ros/ros.h"
#include <vector>
#include <algorithm>
#include <signal.h>
#include <serial/serial.h>
#include "std_msgs/UInt32MultiArray.h"
#include "sensor_msgs/Imu.h"
//const unsigned char HEADER[2] = {0x55, 0x02};
const unsigned char END[2] = {0x0d, 0x0a};
// Serial_port Object
sensor_msgs::Imu ImuMsg;
std_msgs::UInt32MultiArray EncoderMsg;
const unsigned char HEADER_RECV[2] = {0xaa, 0x01};
const unsigned char HEADER_IMU = 0x03;

serial::Serial sp;  // handle of ROS Serial

std::vector<unsigned char> last_buf;
unsigned int imu_seq=0;
bool is_imu = false;
//define data for receive
union EncoderData
{
    unsigned int value;
    unsigned char data[4];
} left_coderCurr, right_coderCurr;
union ImuData
{
    short value;
    unsigned char data[2];
};

void Serial_Init()
{
    
    serial::Timeout to = serial::Timeout::simpleTimeout(10);//创建timeout
    serial::parity_t pt = serial::parity_t::parity_none;//创建校验位为0位
    serial::bytesize_t bt = serial::bytesize_t::eightbits;//创建发送字节数为8位
    serial::flowcontrol_t ft = serial::flowcontrol_t::flowcontrol_none;//创建数据流控制，不使用
    serial::stopbits_t st = serial::stopbits_t::stopbits_one;//创建终止位为1位
    
    sp.setPort("/dev/ttyUSB0");//设置要打开的串口名称
    sp.setBaudrate(115200);//设置串口通信的波特率
    sp.setParity(pt);//设置校验位
    sp.setBytesize(bt);//设置发送字节数
    sp.setFlowcontrol(ft);//设置数据流控制
    sp.setStopbits(st);//设置终止位
    
    sp.setTimeout(to);//串口设置timeout
 
    try
    {
        //打开串口
        sp.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return;
    }
}

void Build_Rosmsg(){
    ImuMsg.header.seq = imu_seq;
    ImuMsg.header.frame_id = "base_link";
    ImuMsg.header.stamp = ros::Time::now();
    ImuMsg.orientation.x=0;
    ImuMsg.orientation.y=0;
    ImuMsg.orientation.z=0;
    ImuMsg.orientation.w=1;
    ImuMsg.orientation_covariance={-1,-1,-1,-1,-1,-1,-1,-1,-1};
    ImuMsg.linear_acceleration.x = 0;
    ImuMsg.linear_acceleration.y = 0;
    ImuMsg.linear_acceleration.z = 0;
    ImuMsg.linear_acceleration_covariance={-1,-1,-1,-1,-1,-1,-1,-1,-1};
    ImuMsg.angular_velocity.x = 0;
    ImuMsg.angular_velocity.y = 0;
    ImuMsg.angular_velocity.z = 0;
    ImuMsg.angular_velocity_covariance={-1,-1,-1,-1,-1,-1,-1,-1,-1};;
}

bool receiveStatus(){
//bool receiveStatus(double &vel_x, double &vel_angle_z, double &angle, std_msgs::Int8MultiArray &serial_data, unsigned char &ctrlFlag)
    //ROS_INFO(" +++++++++++ receiveStatus +++++++++++ ");
    // function: read buffer at 100Hz, while the uploading freq is 10Hz

    unsigned char buf_from_read[5000] = {0};
    std::vector<unsigned char> buf;
    size_t last_seq_len = 0;
    size_t buf_len = 0;
    size_t n = 0;
    
    size_t Receive_N = sp.available();//获取缓冲区内的字节数

    //move last_buf to buf, buf = last_buf
    last_seq_len = last_buf.size();
    if(last_seq_len>0){
        buf.insert(buf.end(),last_buf.begin(),last_buf.end());
    }

    //ROS_INFO("buffer found %ld bytes ", Receive_N);
    if(Receive_N != 0){
        n = sp.read(buf_from_read, Receive_N );//从serial port读出数据到buf_from_read
        
        if(Receive_N>150){
	    return true;
	}
        //buf_from_read print
        /*for (int i = 0; i < Receive_N; i++) {
            std::cout << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (buf_from_read[i] & 0xff) << " ";
        }
        std::cout << std::endl;*/
	
        //move buf_from_read to buf, buf = last_buf + buf_from_read
        for(int i=0;i<Receive_N;i++){
            buf.emplace_back(buf_from_read[i]);
        }
        memset(buf_from_read,0,sizeof(buf_from_read));// clear buf_from_read
    }
        
    buf_len = buf.size();// size of buf

    //ROS_INFO("buf_len = %d",buf_len);
    auto header_pos = std::find(buf.begin(),buf.end(),HEADER_RECV[0]);// find aa
    while(header_pos!=buf.end()){
        size_t pos = std::distance(buf.begin(),header_pos);
        if(buf[pos+1]==HEADER_RECV[1]){// find aa 01
            //look 17 B after pos
            if(pos+16<buf_len){
                if(buf[pos+15]==END[0]&&buf[pos+16]==END[1]){
                    // 读取phase值, unsigned int占4B
                    for (int i = pos; i < pos + 4; i++) {
                        left_coderCurr.data[i] = buf[i + 2];
                        right_coderCurr.data[i] = buf[i + 6];
                    }
                    
                    ROS_INFO("Left_wheel encoder is: %u", left_coderCurr.value);
                    ROS_INFO("Right_wheel encoder is: %u", right_coderCurr.value);
                                    
                    //EncoderMsg初始化和赋值
                    if(EncoderMsg.data.size()==0){
                        EncoderMsg.data.emplace_back(left_coderCurr.value);
                        EncoderMsg.data.emplace_back(right_coderCurr.value);
                    }else{
                        EncoderMsg.data[0]=left_coderCurr.value;
                        EncoderMsg.data[1]=right_coderCurr.value;
                    }
                    //prepare last_buf for next cycle
                    std::vector<unsigned char> rest_buff;
                    rest_buff.insert(rest_buff.end(),header_pos+17,buf.end());
                    swap(rest_buff,last_buf);
                } // end with 0d 0a, and msg len == 17
                last_buf.clear();
                return true;
            }else{
                //need to move buffer to last_buf for next cycle
                std::vector<unsigned char> rest_buff;
                rest_buff.insert(rest_buff.end(),header_pos,buf.end());
                swap(rest_buff,last_buf);
                return true;
            }
        }else{
            auto cur_pos = header_pos;
            header_pos = std::find(cur_pos+1,buf.end(),HEADER_RECV[0]);
        }
    }
    //header not found, prepare for next cycle
    last_buf.clear();    

    //ROS_INFO("------------- receiveStatus ------------");

    return true;

    //float factor = 100.0;

    //    // IMU data
    //    ImuData acc_x, acc_y, acc_z;             // 加速度
    //    ImuData v_angle_x, v_angle_y, v_angle_z; // 角速度

    //Receive data from serial and allocate space
    //Iterate buff and log_info
   

        //Check data_current by crc
        //    unsigned char crc16_check[2] = {0};
        //    uint8_t crc_check_len = 13;
        //    if (buf[1] == HEADER_IMU)
        //    {
        //        crc_check_len = 26;
        //        is_imu = true;
        //    }

        //    getCrc16(buf, crc_check_len, crc16_check);
        //    if (crc16_check[0] != buf[crc_check_len] || crc16_check[1] != buf[crc_check_len + 1])
        //    {
        //        ROS_ERROR("Received data check sum error!");
        //        free(buf);
        //        buf = nullptr;
        //        return false;
        //    }

        // 读取控制标志位

        //if (is_imu) { // 解析 IMU data

            //    {
            //        for (i = 0; i < 2; i++)
            //        {
            //            acc_x.data[i] = buf[i + 2];
            //            acc_y.data[i] = buf[i + 4];
            //            acc_z.data[i] = buf[i + 6];
            //        }
            //        for (i = 0; i < 2; i++)
            //        {
            //            v_angle_x.data[i] = buf[i + 8];
            //            v_angle_y.data[i] = buf[i + 10];
            //            v_angle_z.data[i] = buf[i + 12];
            //        }
            //
            //        ROS_INFO("acc_x=%d, acc_y=%d, acc_z=%d", acc_x.value, acc_y.value, acc_z.value);
            //        ROS_INFO("v_angle_x=%d, v_angle_y=%d, v_angle_z=%d", v_angle_x.value, v_angle_y.value, v_angle_z.value);
            //
//            ImuMsg.header.seq=imu_seq;
//            ImuMsg.header.stamp = ros::Time::now();
//            ImuMsg.linear_acceleration.x = acc_x.value / factor;
//            ImuMsg.linear_acceleration.y = acc_y.value / factor;
//            ImuMsg.linear_acceleration.z = acc_z.value / factor;
//            ImuMsg.angular_velocity.x = v_angle_x.value / factor;
//            ImuMsg.angular_velocity.y = v_angle_y.value / factor;
//            ImuMsg.angular_velocity.z = v_angle_z.value / factor;
//            imu_seq+=1;
// }
//}
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--)
    {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 *bufData：指令数据
 *buflen：处理的指令长度
 *pcrc：处理完之后的CRC码
 **/
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

static void closeSerial(int n){
    sp.close();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "receive_serial_node");
    ros::NodeHandle node;
    ros::Publisher encoder_pub;
    ros::Publisher imu_pub;

    Serial_Init();
    Build_Rosmsg();
    signal(SIGINT,closeSerial);
    ros::Rate loop_rate(100);
    encoder_pub = node.advertise<std_msgs::UInt32MultiArray>("serial_encoder",2);
    imu_pub = node.advertise<sensor_msgs::Imu>("/imu/data_raw",2);
    while(receiveStatus()){
        if(ros::ok){
            //imu_pub.publish(ImuMsg);
            encoder_pub.publish(EncoderMsg);
        }
        loop_rate.sleep();
    }
    
}
