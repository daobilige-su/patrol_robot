#! /usr/bin/env python
import rospy                                    #ros in python
import tf
import math
import numpy as np
from geometry_msgs.msg import Vector3Stamped    #geometry_msgs/Vector3Stamped:a std_msg in ros
from sensor_msgs.msg import PointCloud          #PointCloud:sensor_msgs in ros
from sensor_msgs.msg import LaserScan           #LaserScan:sensor_msgs in ros
from sensor_msgs.msg import Imu                 #Imu:sensor_msgs in ros
import message_filters                          #msg manager in ros

pub = rospy.Publisher("/imu_euler_data", Vector3Stamped, queue_size=1)
def Imu_callback(msg):
    #rospy.loginfo(msg)
    flt_roll = msg.vector.x
    flt_pitch = msg.vector.y
    flt_yaw = msg.vector.z                                                      ##Get euler_angle from msg
    old_matrix=tf.transformations.euler_matrix(flt_roll,flt_pitch,flt_yaw,axes='sxyz')  ##euerl_to_matrix
    orientation=tf.transformations.euler_matrix(math.pi,0,0,axes='sxyz')        ##(z180)orientation matrix
    new_matrix=np.matmul(orientation,old_matrix)                                ##Orientation from reference matrix
    new_euler=tf.transformations.euler_from_matrix(new_matrix,axes='sxyz')      ##Martix to euler
    #rospy.loginfo(new_euler)
    imu_msg=Vector3Stamped()
    #imu_msg.stamp=rospy.time()
    imu_msg.vector.x = new_euler[0]
    imu_msg.vector.y = new_euler[1]
    imu_msg.vector.z = new_euler[2]
    pub.publish(imu_msg)

def pointcld_to_laserscan():
    rospy.init_node("imu_frame_change",anonymous=True)                           ##  init node
    rospy.loginfo("Node of imu_filter is launch, sending Euler_angle.")
    rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, Imu_callback)
    rospy.spin()

if __name__ =="__main__":
    pointcld_to_laserscan()
