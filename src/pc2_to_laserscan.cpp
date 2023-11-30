#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher laser_pub;//全局变量Pub

// ROS Parameters
//target_frame: laser_link # Leave disabled to output scan in pointcloud frame
//transform_tolerance: 0.01
//min_height: -0.465
//max_height: 1
//
//angle_min: -3.14# -M_PI -3.14
//angle_max: 3.14# M_PI 0.18
//angle_increment: 0.003 # 0.17degree
//scan_time: 0.1
//range_min: 0.2
//range_max: 100
//use_inf: true
//inf_epsilon: 1.0

std::string target_frame_ = "laser_link";
double tolerance_;
double min_height_=-0.465, max_height_=1, angle_min_=-3.14, angle_max_=3.14, angle_increment_=0.003, scan_time_=0.1, range_min_=0.2, range_max_=100;
bool use_inf_=true;
double inf_epsilon_=1.0;

void vpl16Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // build laserscan output
    sensor_msgs::LaserScan laser_msg;
    laser_msg.header = cloud_msg->header;

    laser_msg.angle_min = angle_min_;
    laser_msg.angle_max = angle_max_;
    laser_msg.angle_increment = angle_increment_;
    laser_msg.time_increment = 0.0;
    laser_msg.scan_time = scan_time_;
    laser_msg.range_min = range_min_;
    laser_msg.range_max = range_max_;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {laser_msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());}
    else
    {laser_msg.ranges.assign(ranges_size, laser_msg.range_max + inf_epsilon_);}

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"),
       iter_z(*cloud_msg, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {continue;}

        if (*iter_z > max_height_ || *iter_z < min_height_)
        {continue;}

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min_)
        {continue;}
        if (range > range_max_)
        {continue;}

        double angle = atan2(*iter_y, *iter_x);
        if (angle < laser_msg.angle_min || angle > laser_msg.angle_max)
        {continue;}

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - laser_msg.angle_min) / laser_msg.angle_increment;
        if (range < laser_msg.ranges[index])
        {laser_msg.ranges[index] = range;}
    }
    laser_pub.publish(laser_msg);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc2_to_laserscan_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vpl16", 2, vpl16Callback); //
    laser_pub = node.advertise<sensor_msgs::LaserScan>("scan",2);
    ros::spin();  //ros::spin()库是响应循环，消息到达时调用函数chatterCallback，CTRL+C结束循环
    return 0;
}

