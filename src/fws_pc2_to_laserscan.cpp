#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>
#include "yaml-cpp/yaml.h"
ros::Publisher laser_pub;//全局变量Pub
ros::Publisher laser_mid_pub;//全局变量Pub
ros::Publisher new_cld_pub;

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
// 需要在cmakelist/xml中导入roslib,获取pkg绝对路径

//std::string package_path = ros::package::getPath("patrol_robot");
//std::string yaml_path=package_path+"/param/fws_sim.yaml";
//YAML::Node config_yaml=YAML::LoadFile(param_yaml_file);//调用yaml文件，（需要apt-install:libyaml-cpp-dev并修改camke添加库文件）

//std::string target_frame_ = "laser_link";
//double tolerance_;//ls_z_min=-0.32;sim_z_min=-0.465 ——angle_max0.17
double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;

bool use_inf_=true;
double inf_epsilon_=1.0;

void vlp16Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Eular angle to Rotation_matrix(RPY)
    boost::shared_ptr<geometry_msgs::Vector3Stamped const> imu_msg;
    float roll=0.0,pitch=0.0,yaw=0.0;
    imu_msg = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/imu_euler_data", ros::Duration(0.05));
    if(imu_msg!=NULL){
        roll=imu_msg->vector.x;
        pitch=imu_msg->vector.y;
        //yaw=imu_msg->vector.z;
    }
    Eigen::Vector3f eulerAngle(roll,pitch,yaw);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix=yawAngle*pitchAngle*rollAngle;


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

    // build laserscan using middle ring scan output
    sensor_msgs::LaserScan laser_mid_msg;
    laser_mid_msg.header = cloud_msg->header;
    laser_mid_msg.angle_min = angle_min_;
    laser_mid_msg.angle_max = angle_max_;
    laser_mid_msg.angle_increment = angle_increment_;
    laser_mid_msg.time_increment = 0.0;
    laser_mid_msg.scan_time = scan_time_;
    laser_mid_msg.range_min = range_min_;
    laser_mid_msg.range_max = range_max_;

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {laser_mid_msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());}
    else
    {laser_mid_msg.ranges.assign(ranges_size, laser_mid_msg.range_max + inf_epsilon_);}

    std::vector<float> array={};
    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"),
       iter_z(*cloud_msg, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        //Imu_filter
        Eigen::Vector3f old_point(*iter_x,*iter_y,*iter_z);
        Eigen::Vector3f Imu_point=rotation_matrix*old_point; // rotate with imu data
        float pt_x = *iter_x;
        float pt_y = *iter_y;
        float pt_z = *iter_z;

        float imu_x = *iter_x;
        float imu_y = *iter_y;
        float imu_z = *iter_z;
        // points rotated by imu data
        if(Imu_point.size()==3) {
            imu_x = Imu_point[0];
            imu_y = Imu_point[1];
            imu_z = Imu_point[2];
        }
        array.push_back(imu_x);
        array.push_back(imu_y);
        array.push_back(imu_z);
        //std::cout<<Imu_point.size()<<"  "<<imu_msg->vector<<std::endl;
        if (std::isnan(imu_x) || std::isnan(imu_y) || std::isnan(imu_z))
        {continue;}

        if (imu_z > max_height_ || imu_z < min_height_)
        {continue;}

        double range = hypot(imu_x, imu_y);
        if (range < range_min_)
        {continue;}
        if (range > range_max_)
        {continue;}

        double angle = atan2(imu_y, imu_x);
        if (angle < laser_msg.angle_min || angle > laser_msg.angle_max)
        {continue;}

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - laser_msg.angle_min) / laser_msg.angle_increment;
        if (range < laser_msg.ranges[index])
        {laser_msg.ranges[index] = range;}

        // laser_mid_msg
        float pitch_angle = atan2(pt_z, range)*(180.0/M_PI);
        if (abs(pitch_angle-1)<0.5)
        {
            if (range < laser_mid_msg.ranges[index])
            {laser_mid_msg.ranges[index] = range;}
        }
    }
    laser_pub.publish(laser_msg);
    laser_mid_pub.publish(laser_mid_msg);
    //std::cout<<array.size()<<"  ?  "<<cloud_msg->width<<std::endl;

    //The code is pub new_cloud_msg(rotation by imu) to test the imu_msg;
    sensor_msgs::PointCloud2 new_cld_msg;
    new_cld_msg.header = cloud_msg->header;
    new_cld_msg.width =  cloud_msg->width;
    new_cld_msg.height = cloud_msg->height;
    new_cld_msg.fields = cloud_msg->fields;
    new_cld_msg.is_dense = cloud_msg->is_dense;
    new_cld_msg.point_step = cloud_msg->point_step;
    new_cld_msg.row_step = cloud_msg->row_step;
    new_cld_msg.data.resize(new_cld_msg.row_step*new_cld_msg.height);
    unsigned int offset = 0;
    for (unsigned int i = 0; i < new_cld_msg.width; i++) {
        memcpy(&new_cld_msg.data[offset + 0], &array[3*i+0], sizeof(array[0]));
        memcpy(&new_cld_msg.data[offset + 4], &array[3*i+1], sizeof(array[0]));
        memcpy(&new_cld_msg.data[offset + 8], &array[3*i+2], sizeof(array[0]));
        offset += new_cld_msg.point_step;
    }
    new_cld_pub.publish(new_cld_msg);
}

int main(int argc, char **argv)
{
    // init ros node before calling ros param
    ros::init(argc, argv, "pc2_to_laserscan_node");
    ros::NodeHandle node;

    std::string param_yaml_file;
    ros::param::get("/param_yaml_file", param_yaml_file);
    //std::string package_path = ros::package::getPath("patrol_robot");
    //std::string yaml_path=package_path+"/param/fws_sim.yaml";
    std::cout<<param_yaml_file.c_str()<<std::endl;
    YAML::Node config_yaml=YAML::LoadFile(param_yaml_file);//调用yaml文件，（需要apt-install:libyaml-cpp-dev并修改camke添加库文件）

    //std::string target_frame_ = "laser_link";
    //double tolerance_;//ls_z_min=-0.32;sim_z_min=-0.465 ——angle_max0.17
    min_height_=config_yaml["laser"]["min_height"].as<float>();
    max_height_=config_yaml["laser"]["max_height"].as<float>();
    angle_min_=config_yaml["laser"]["angle_min"].as<float>();
    angle_max_=config_yaml["laser"]["angle_max"].as<float>();
    angle_increment_=config_yaml["laser"]["angle_increment"].as<float>();
    scan_time_=config_yaml["laser"]["scan_time"].as<float>();
    range_min_=config_yaml["laser"]["range_min"].as<float>();
    range_max_=config_yaml["laser"]["range_max"].as<float>();


    std::cout<< "Getting param from yaml_file:"<<param_yaml_file<<std::endl;
    std::cout<< "min_height="<<min_height_<< std::endl;
    std::cout<< "max_height="<<max_height_<< std::endl;
    std::cout<< "angle_min="<<angle_min_<< std::endl;
    std::cout<< "angle_max="<<angle_max_<< std::endl;
    std::cout<< "angle_increment="<<angle_increment_<< std::endl;
    std::cout<< "scan_time="<<scan_time_<< std::endl;
    std::cout<< "range_min="<<range_min_<< std::endl;
    std::cout<< "range_max="<<range_max_<< std::endl;

    ros::Subscriber sub = node.subscribe("/vlp16", 2, vlp16Callback); //
    laser_pub = node.advertise<sensor_msgs::LaserScan>("/scan",2);
    laser_mid_pub = node.advertise<sensor_msgs::LaserScan>("/scan_mid",2);
    new_cld_pub = node.advertise<sensor_msgs::PointCloud2>("/imu_vlp16",2);
    ros::spin();  //ros::spin()库是响应循环，消息到达时调用函数chatterCallback，CTRL+C结束循环
    return 0;
}

