#! /usr/bin/env python

import numpy as np
import math
# from __future__ import print_function

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
import rospy
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from tf import transformations
import tf
import yaml


class FwsModel:
    def __init__(self):
        # load params
        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        # geometric params & robot status
        self.wheel_diameter = self.param['base']['wheel_diameter']
        self.L = self.param['base']['L']  # FL wheel x coord
        self.W = self.param['base']['W']  # FL wheel y coord
        self.d = self.param['base']['d']  # deviation
        self.steer_theta_min = self.param['base']['steer_theta_min'] * (math.pi / 180.0)
        self.steer_theta_max = self.param['base']['steer_theta_max'] * (math.pi / 180.0)

        self.pose = np.zeros((3, 1))

        self.steer_motor_theta = np.zeros((4, 1))
        self.drive_motor_theta = np.zeros((4, 1))

        # publishers & subscribers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.fws_cmd_pub = rospy.Publisher('fws_cmd', Float32MultiArray, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.compute_fws_cmd)
        self.fws_meas_sub = rospy.Subscriber("fws_meas", Float32MultiArray, self.compute_odom)

        self.br = tf.TransformBroadcaster()

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)


    def compute_fws_cmd(self, data):

        V1 = data.linear.x
        V2 = data.linear.y
        V3 = data.angular.z

        dtheta1 = 0
        dtheta2 = 0
        dtheta3 = 0
        dtheta4 = 0

        # Eq 12
        x1 = self.L
        y1 = self.W
        x2 = self.L
        y2 = -1 * self.W
        x3 = -1 * self.L
        y3 = self.W
        x4 = -1 * self.L
        y4 = -1 * self.W

        d = self.d

        # Eq 6(1st 2 terms)
        v1x_sw = V1 + (-1) * V3 * y1
        v1y_sw = V2 + V3 * x1
        v2x_sw = V1 + (-1) * V3 * y2
        v2y_sw = V2 + V3 * x2
        v3x_sw = V1 + (-1) * V3 * y3
        v3y_sw = V2 + V3 * x3
        v4x_sw = V1 + (-1) * V3 * y4
        v4y_sw = V2 + V3 * x4

        # Eq 8(1st 2 terms)
        theta1 = math.atan2(v1y_sw, v1x_sw)
        theta2 = math.atan2(v2y_sw, v2x_sw)
        theta3 = math.atan2(v3y_sw, v3x_sw)
        theta4 = math.atan2(v4y_sw, v4x_sw)

        # Eq 13
        a1 = -1 * d * math.sin(theta1)
        b1 = d * math.cos(theta1)
        a2 = d * math.sin(theta2)
        b2 = -1 * d * math.cos(theta2)
        a3 = -1 * d * math.sin(theta3)
        b3 = d * math.cos(theta3)
        a4 = d * math.sin(theta4)
        b4 = -1 * d * math.cos(theta4)

        # Eq 14
        v1x = 1 * V1 + 0 * V2 + (-1) * (y1 + b1) * V3 + (-1) * b1 * dtheta1
        v1y = 0 * V1 + 1 * V2 + (x1 + a1) * V3 + a1 * dtheta1

        v2x = 1 * V1 + 0 * V2 + (-1) * (y2 + b2) * V3 + (-1) * b2 * dtheta2
        v2y = 0 * V1 + 1 * V2 + (x2 + a2) * V3 + a2 * dtheta2

        v3x = 1 * V1 + 0 * V2 + (-1) * (y3 + b3) * V3 + (-1) * b3 * dtheta3
        v3y = 0 * V1 + 1 * V2 + (x3 + a3) * V3 + a3 * dtheta3

        v4x = 1 * V1 + 0 * V2 + (-1) * (y4 + b4) * V3 + (-1) * b4 * dtheta4
        v4y = 0 * V1 + 1 * V2 + (x4 + a4) * V3 + a4 * dtheta4

        # wheel wc
        wc1 = math.sqrt(math.pow(v1x, 2) + math.pow(v1y, 2)) / (self.wheel_diameter / 2.0)
        wc2 = math.sqrt(math.pow(v2x, 2) + math.pow(v2y, 2)) / (self.wheel_diameter / 2.0)
        wc3 = math.sqrt(math.pow(v3x, 2) + math.pow(v3y, 2)) / (self.wheel_diameter / 2.0)
        wc4 = math.sqrt(math.pow(v4x, 2) + math.pow(v4y, 2)) / (self.wheel_diameter / 2.0)

        # ensure minimum steering angle change
        theta1_pre = self.steer_motor_theta[0, 0]
        theta2_pre = self.steer_motor_theta[1, 0]
        theta3_pre = self.steer_motor_theta[2, 0]
        theta4_pre = self.steer_motor_theta[3, 0]

        theta1, wc1 = self.compute_optimum_steering(theta1, theta1_pre, wc1, self.steer_theta_min, self.steer_theta_max)
        theta2, wc2 = self.compute_optimum_steering(theta2, theta2_pre, wc2, self.steer_theta_min, self.steer_theta_max)
        theta3, wc3 = self.compute_optimum_steering(theta3, theta3_pre, wc3, self.steer_theta_min, self.steer_theta_max)
        theta4, wc4 = self.compute_optimum_steering(theta4, theta4_pre, wc4, self.steer_theta_min, self.steer_theta_max)

        # send fws cmd
        fws_cmd = np.array([theta1, theta2, theta3, theta4, wc1, wc2, wc3, wc4])
        fws_cmd_msg = Float32MultiArray(data=fws_cmd.tolist())

        self.fws_cmd_pub.publish(fws_cmd_msg)

        return True

    @staticmethod
    def compute_optimum_steering(theta, theta_pre, wc, theta_min, theta_max):
        if theta>0:
            theta_alter = theta-math.pi
        else:
            theta_alter = theta+math.pi

        theta_cha = theta-theta_pre
        theta_alter_cha = theta_alter-theta_pre

        if abs(theta_alter_cha)<abs(theta_cha):
            theta = theta_alter
            wc = -1*wc

        # for non-360 deg steering wheel
        if theta<theta_min:
            theta = theta+math.pi
            wc = -1*wc
        elif theta>theta_max:
            theta = theta-math.pi
            wc = -1*wc

        return theta, wc

    def compute_odom(self, data):

        odom_dt = 0.1 # TODO
        d = self.d
        # Eq 12
        x1 = self.L
        y1 = self.W
        x2 = self.L
        y2 = -1 * self.W
        x3 = -1 * self.L
        y3 = self.W
        x4 = -1 * self.L
        y4 = -1 * self.W

        Px = self.pose[0, 0]
        Py = self.pose[1, 0]
        Pth = self.pose[2, 0]

        FLSteerMotor_joint_ang_cur = data.data[0]
        FLDriveMotor_joint_ang_cur = data.data[1]
        FRSteerMotor_joint_ang_cur = data.data[2]
        FRDriveMotor_joint_ang_cur = data.data[3]
        BLSteerMotor_joint_ang_cur = data.data[4]
        BLDriveMotor_joint_ang_cur = data.data[5]
        BRSteerMotor_joint_ang_cur = data.data[6]
        BRDriveMotor_joint_ang_cur = data.data[7]

        FLDriveMotor_joint_ang_pre = self.drive_motor_theta[0, 0]
        FRDriveMotor_joint_ang_pre = self.drive_motor_theta[1, 0]
        BLDriveMotor_joint_ang_pre = self.drive_motor_theta[2, 0]
        BRDriveMotor_joint_ang_pre = self.drive_motor_theta[3, 0]

        # compute drive motors' rotation
        BLDriveMotor_joint_ang_change = 0
        BRDriveMotor_joint_ang_change = 0
        FLDriveMotor_joint_ang_change = 0
        FRDriveMotor_joint_ang_change = 0

        # if joint angle is only from -180 to 180, and there is NO larger than 180 deg rot in one odom sample
        if abs(BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre)<math.pi:
            BLDriveMotor_joint_ang_change = BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre
        elif (BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre)<0:
            BLDriveMotor_joint_ang_change = BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre + math.pi * 2
        elif (BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre)>0:
            BLDriveMotor_joint_ang_change = BLDriveMotor_joint_ang_cur - BLDriveMotor_joint_ang_pre - math.pi * 2

        if abs(BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre) < math.pi:
            BRDriveMotor_joint_ang_change = BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre
        elif (BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre) < 0:
            BRDriveMotor_joint_ang_change = BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre + math.pi * 2
        elif (BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre) > 0:
            BRDriveMotor_joint_ang_change = BRDriveMotor_joint_ang_cur - BRDriveMotor_joint_ang_pre - math.pi * 2

        if abs(FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre) < math.pi:
            FLDriveMotor_joint_ang_change = FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre
        elif (FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre) < 0:
            FLDriveMotor_joint_ang_change = FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre + math.pi * 2
        elif (FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre) > 0:
            FLDriveMotor_joint_ang_change = FLDriveMotor_joint_ang_cur - FLDriveMotor_joint_ang_pre - math.pi * 2

        if abs(FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre) < math.pi:
            FRDriveMotor_joint_ang_change = FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre
        elif (FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre) < 0:
            FRDriveMotor_joint_ang_change = FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre + math.pi * 2
        elif (FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre) > 0:
            FRDriveMotor_joint_ang_change = FRDriveMotor_joint_ang_cur - FRDriveMotor_joint_ang_pre - math.pi * 2

        # remark steering angles
        theta1 = FLSteerMotor_joint_ang_cur
        theta2 = FRSteerMotor_joint_ang_cur
        theta3 = BLSteerMotor_joint_ang_cur
        theta4 = BRSteerMotor_joint_ang_cur
        dtheta1 = 0
        dtheta2 = 0
        dtheta3 = 0
        dtheta4 = 0

        # compute drive motor vel
        v1x = (FLDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.cos(FLSteerMotor_joint_ang_cur)
        v1y = (FLDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.sin(FLSteerMotor_joint_ang_cur)
        v2x = (FRDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.cos(FRSteerMotor_joint_ang_cur)
        v2y = (FRDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.sin(FRSteerMotor_joint_ang_cur)
        v3x = (BLDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.cos(BLSteerMotor_joint_ang_cur)
        v3y = (BLDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.sin(BLSteerMotor_joint_ang_cur)
        v4x = (BRDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.cos(BRSteerMotor_joint_ang_cur)
        v4y = (BRDriveMotor_joint_ang_change * (self.wheel_diameter / 2.0) / odom_dt) * math.sin(BRSteerMotor_joint_ang_cur)

        # Eq 13
        a1 = -1 * d * math.sin(theta1)
        b1 = d * math.cos(theta1)
        a2 = d * math.sin(theta2)
        b2 = -1 * d * math.cos(theta2)
        a3 = -1 * d * math.sin(theta3)
        b3 = d * math.cos(theta3)
        a4 = d * math.sin(theta4)
        b4 = -1 * d * math.cos(theta4)

        # Eq 11 & Eq 14
        t1 = v1x + b1 * dtheta1
        t2 = v1y - a1 * dtheta1
        t3 = v2x + b2 * dtheta2
        t4 = v2y - a2 * dtheta2
        t5 = v3x + b3 * dtheta3
        t6 = v3y - a3 * dtheta3
        t7 = v4x + b4 * dtheta4
        t8 = v4y - a4 * dtheta4

        h1 = -(y1 + b1)
        h2 = (x1 + a1)
        h3 = -(y2 + b2)
        h4 = (x2 + a2)
        h5 = -(y3 + b3)
        h6 = (x3 + a3)
        h7 = -(y4 + b4)
        h8 = (x4 + a4)

        vc = np.array([t1, t2, t3, t4, t5, t6, t7, t8]).reshape((8, 1))
        H = np.array([[1, 0, h1], [0, 1, h2], [1, 0, h3], [0, 1, h4], [1, 0, h5], [0, 1, h6], [1, 0, h7], [0, 1, h8]])

        # V = H:pinv(vc_) - - m: pinv(b)
        V = np.linalg.pinv(H) @ vc

        V1 = V[0, 0]
        V2 = V[1, 0]
        V3 = V[2, 0]

        # compute odom
        delta_x = (V1 * math.cos(Pth) - V2 * math.sin(Pth)) * odom_dt
        delta_y = (V1 * math.sin(Pth) + V2 * math.cos(Pth)) * odom_dt
        delta_th = V3 * odom_dt

        if delta_th > math.pi:
            delta_th = delta_th - 2 * math.pi
        elif delta_th < (-1) * math.pi:
            delta_th = delta_th + 2 * math.pi

        Px = Px + delta_x
        Py = Py + delta_y
        Pth = Pth + delta_th

        if Pth > math.pi:
            Pth = Pth - 2 * math.pi
        elif Pth < (-1) * math.pi:
            Pth = Pth + 2 * math.pi

        # axis angle to quatenion: rotate around(_x, _y, _z) axis with p angle:
        Pth_quat = [0, 0, math.sin(Pth / 2.0), math.cos(Pth / 2.0)]

        # publish odom msg
        odom_msg = Odometry()
        odom_msg.header.seq = 0 # TODO
        odom_msg.header.stamp = rospy.get_rostime()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = Px
        odom_msg.pose.pose.position.y = Py
        odom_msg.pose.pose.orientation.z = Pth_quat[2]
        odom_msg.pose.pose.orientation.w = Pth_quat[3]

        self.odom_pub.publish(odom_msg)

        # publish odom tf
        self.tf_broadcast([Px, Py, 0.0], [0.0, 0.0, Pth_quat[2], Pth_quat[3]], 'base_link', 'odom')

        # restore results
        self.pose = np.array([Px, Py, Pth]).reshape((3, 1))
        self.drive_motor_theta = np.array([FLDriveMotor_joint_ang_cur, FRDriveMotor_joint_ang_cur, BLDriveMotor_joint_ang_cur,
                                           BRDriveMotor_joint_ang_cur]).reshape((4, 1))
        self.steer_motor_theta = np.array([FLSteerMotor_joint_ang_cur, FRSteerMotor_joint_ang_cur, BLSteerMotor_joint_ang_cur,
                                           BRSteerMotor_joint_ang_cur]).reshape((4, 1))


def main(args):
    rospy.init_node('fws_model_node', anonymous=True)
    model = FwsModel()
    rospy.spin()
    # try:
    #     while not rospy.is_shutdown():
    #         # model.do_something()
    #         rospy.sleep(0.2)
    # except KeyboardInterrupt:
    #     print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)