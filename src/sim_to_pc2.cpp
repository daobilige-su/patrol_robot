#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <vector> //使用vector时需要的头文件

ros::Publisher cloud_pub;//全局变量Pub

void vpl16Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> array = msg->data;//获取数据
    unsigned int size=array.size();//计算长度
    unsigned int point_num=size/3;

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "laser_link";//填充 PointCloud 消息的头：frame 和 timestamp

    cloud.height = 1;
    cloud.width = point_num;
    cloud.fields.resize(3);

    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].count = 1;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].count = 1;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].count = 1;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
    cloud.is_dense = false;

    unsigned int offset = 0;
    for (unsigned int i = 0; i < point_num; i++) {
        memcpy(&cloud.data[offset + 0], &array[3*i+2], sizeof(array[0]));
        memcpy(&cloud.data[offset + 4], &array[3*i+0], sizeof(array[0]));
        memcpy(&cloud.data[offset + 8], &array[3*i+1], sizeof(array[0]));
        offset += cloud.point_step;
    }

    cloud_pub.publish(cloud);//发布点云信息
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_array_to_pc2_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vlp16_floatarray", 2, vpl16Callback); //
    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("vlp16",2);
    ros::spin();  //ros::spin()库是响应循环，消息到达时调用函数chatterCallback，CTRL+C结束循环
    return 0;
}

