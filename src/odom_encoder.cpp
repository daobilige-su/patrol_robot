// Created by zzs on 24-1-8.
#include "ros/ros.h"
#include <ros/time.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt32MultiArray.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>

const double pi=std::acos(-1);
////Get param(wheel diameter and dist) from the yaml
std::string package_path = ros::package::getPath("patrol_robot");
std::string yaml_path=package_path+"/param/ptcld_to_scan.yaml";
YAML::Node config_yaml=YAML::LoadFile(yaml_path);//调用yaml文件，（需要apt-install:libyaml-cpp-dev并修改camke添加库文件）
const double wheel_diameter=config_yaml["wheel_diameter"].as<double>()/1000,wheel_dist=config_yaml["wheel_dist"].as<double>()/1000,wheel_len_dist=config_yaml["wheel_len_dist"].as<double>()/1000;

unsigned int odom_seq=0,tf_odom_seq=0;
unsigned int encoder_array[4]={0,0,0,0};
double time_begin;
ros::Publisher odom_pub;
nav_msgs::Odometry OdomMsg;
geometry_msgs::TransformStamped tf_odom_msg;
double odomX,odomY,odomTh;


//build odometry_msg
void build_odom_tf_msg(){
   //Bulid rosmsg nav_msgs/Odometry
    OdomMsg.header.seq=0;
    OdomMsg.header.stamp=ros::Time::now();
    OdomMsg.header.frame_id="odom";
    OdomMsg.child_frame_id="base_link";
    OdomMsg.pose.pose.position.x=0;
    OdomMsg.pose.pose.position.y=0;
    OdomMsg.pose.pose.position.z=0;
    OdomMsg.pose.pose.orientation.x=0;
    OdomMsg.pose.pose.orientation.y=0;
    OdomMsg.pose.pose.orientation.z=0;
    OdomMsg.pose.pose.orientation.w=0;
    OdomMsg.pose.covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OdomMsg.twist.twist.linear.x=0;
    OdomMsg.twist.twist.linear.y=0;
    OdomMsg.twist.twist.linear.z=0;
    OdomMsg.twist.twist.angular.x=0;
    OdomMsg.twist.twist.angular.y=0;
    OdomMsg.twist.twist.angular.z=0;
    OdomMsg.twist.covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    //Bulid rosmsg geometry_msgs/TransformStamped as tf msg.
    tf_odom_msg.header.seq=0;
    tf_odom_msg.header.stamp=ros::Time::now();
    tf_odom_msg.header.frame_id="odom";
    tf_odom_msg.child_frame_id="base_link";
    tf_odom_msg.transform.translation.x=0;
    tf_odom_msg.transform.translation.y=0;
    tf_odom_msg.transform.translation.z=0;
    tf_odom_msg.transform.rotation.x=0;
    tf_odom_msg.transform.rotation.y=0;
    tf_odom_msg.transform.rotation.z=0;
    tf_odom_msg.transform.rotation.w=0;
};

//compute odom and send msg
void send_odometry(){
//Calculate odometry (translation in x and y ;rotation theta along z)
    double time_now=ros::Time::now().toSec();
    double odom_dt=time_now-time_begin;
    double ang_accuracy=pow(2,17)*50;   //17bit_encoder and i=50;
    double lwmotor_joint_ang_cur=encoder_array[2];
    double rwmotor_joint_ang_cur=encoder_array[3];
    double lwmotor_joint_ang_pre,rwmotor_joint_ang_pre;

    //Filter the data_init
    if(encoder_array[0]<1&&encoder_array[1]<1){
	lwmotor_joint_ang_pre=encoder_array[2];
	rwmotor_joint_ang_pre=encoder_array[3];
    }else{
    	lwmotor_joint_ang_pre=encoder_array[0];
	    rwmotor_joint_ang_pre=encoder_array[1];
    }
    //Get angle_change(rotation of wheel) from encoder_msg
    double motor_joint_ang_change[2]={0,0}; //array to save ang_change
    motor_joint_ang_change[0]= (lwmotor_joint_ang_cur-lwmotor_joint_ang_pre)/ang_accuracy*2*pi;
    motor_joint_ang_change[1]= (rwmotor_joint_ang_cur-rwmotor_joint_ang_pre)/ang_accuracy*2*pi;
    std::cout<<"Motor_ang_change: "<< motor_joint_ang_change[0]<<"  "<<motor_joint_ang_change[1]<<std::endl;
    double lwmotor_joint_ang_change=0,rwmotor_joint_ang_change=0;
    if(std::abs(motor_joint_ang_change[0])<100*pi && std::abs(motor_joint_ang_change[1])<100*pi){ //compute angle>2*pi
        lwmotor_joint_ang_change=motor_joint_ang_change[0];
        rwmotor_joint_ang_change=motor_joint_ang_change[1];
    }//Filter the data which is discontinuous

    //Compute dx and dtheta
    double lw_rot = lwmotor_joint_ang_change*(wheel_diameter/2.0);
    double rw_rot = -1.0*rwmotor_joint_ang_change*(wheel_diameter/2.0);
    double dx     = (lw_rot+rw_rot)/2.0;
    //ROS_INFO("The delta dx=,%f",dx);
    double dtheta = (rw_rot-lw_rot)*wheel_dist/(wheel_dist*wheel_dist+wheel_len_dist*wheel_len_dist);//w=(vr-vl)*(dist/2)/2/(length*length/4+dist*dist/4);
    // compute X,Y,Theta of wheel encoder based odometry
    odomTh = odomTh+dtheta;
    if (odomTh>pi)
        odomTh=odomTh-(pi*2);
    else if (odomTh<((-1)*pi))
        odomTh=odomTh+(pi*2);
    odomX = odomX+std::cos(odomTh)*dx;
    odomY = odomY+std::sin(odomTh)*dx;
    ROS_INFO("The odom x is=,%f",odomX);
    ROS_INFO("The odom y is=,%f",odomY);
    ROS_INFO("The odom theta is=,%f",odomTh*180/pi);
    // compute velocity vx,w based on wheel encoder
    double lv_meas = (lwmotor_joint_ang_change*(wheel_diameter/2.0))/odom_dt;
    double rv_meas = (rwmotor_joint_ang_change*(wheel_diameter/2.0))/odom_dt;

    double vx_meas = (lv_meas+rv_meas)/2.0;
    double w_meas = ((rv_meas-lv_meas)/2.0)/(wheel_diameter/2.0);
    double odomTh_quat[4]= {0, 0, std::sin(odomTh/2.0), std::cos(odomTh/2.0)};
    //std::cout<<time_begin<<"  "<<time_now<<"  "<<time_now-time_begin<<std::endl;
    //ROS_INFO("The vx_meas dx=,%f",vx_meas);

    //pub tf_odom_msg
    //tf::TransformBroadcaster tf_broad_pub;
    static tf2_ros::TransformBroadcaster tf_broad_pub;
    tf_odom_msg.header.stamp=ros::Time::now();
    tf_odom_msg.header.seq=tf_odom_seq;
    tf_odom_msg.transform.translation.x=odomX;
    tf_odom_msg.transform.translation.y=odomY;//translation in z is always 0.
    tf_odom_msg.transform.rotation.z=odomTh_quat[2];//rotation along x y both is 0.
    tf_odom_msg.transform.rotation.w=odomTh_quat[3];
    tf_broad_pub.sendTransform(tf_odom_msg);

    // pub OdomMsg
    OdomMsg.header.stamp=ros::Time::now();
    OdomMsg.header.seq=odom_seq;
    OdomMsg.pose.pose.position.x=odomX;
    OdomMsg.pose.pose.position.y=odomY;
    OdomMsg.pose.pose.orientation.z=odomTh_quat[2];
    OdomMsg.pose.pose.orientation.w=odomTh_quat[3];
    OdomMsg.twist.twist.linear.x=vx_meas;
    OdomMsg.twist.twist.linear.y=0;
    OdomMsg.twist.twist.linear.z=w_meas;
    odom_pub.publish(OdomMsg);

    //Data of iteration
    odom_seq = odom_seq+1;
    tf_odom_seq=tf_odom_seq+1;
    time_begin=time_now;
    encoder_array[0]=encoder_array[2];
    encoder_array[1]=encoder_array[3];
}

void serialCallback(const std_msgs::UInt32MultiArray::ConstPtr& msg) {
    encoder_array[2] = msg->data[0];
    encoder_array[3] = msg->data[1];
    send_odometry();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_encoder_node");
    ros::NodeHandle node;
    ros::Time::init();
    build_odom_tf_msg();
    odom_pub = node.advertise<nav_msgs::Odometry>("/odom",2);
    ros::Subscriber sub = node.subscribe("serial_encoder", 2, serialCallback); //
    ros::spin();  //ros::spin()库是响应循环，消息到达时调用函数chatterCallback，CTRL+C结束循环
    return 0;
}
