#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import patrol_robot.msg
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import datetime
import time

# params
v_const = 0.15
w_const = 0.15
w_rot = 0.2
jump_t_t = 1.2
jump_t_r = 1.5
unit_t = 0.05

class line_track_action(object):
    _feedback = patrol_robot.msg.line_trackFeedback()
    _result = patrol_robot.msg.line_trackResult()

    def __init__(self):
        self._action_name = 'line_track'
        self._as = actionlib.SimpleActionServer(self._action_name, patrol_robot.msg.line_trackAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        # self.bridge = CvBridge()
        self.line_sensor_data = None
        self.meg_sensor_data_b = None
        self.meg_sensor_sub = rospy.Subscriber("meg_sensor", Int8MultiArray, self.callback_meg_sensor)
        self.meg_sensor_sub_b = rospy.Subscriber("meg_sensor_b", Int8MultiArray, self.callback_meg_sensor_b)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def callback_meg_sensor(self, data):
        self.line_sensor_data = np.array(data.data).reshape((1, -1))

    def callback_meg_sensor_b(self, data):
        self.meg_sensor_data_b = np.array(data.data).reshape((1,-1))

    def execute_cb(self,goal): # 0: stand still, 1: forward; 2, backward; 3, left; 4, right;

        success=False
        msg = Twist()
        move_dir = goal.target_location[0]

        rospy.loginfo('%s: Executing, obtained goal location: (%f)' % (
            self._action_name, goal.target_location[0]))

        if self.line_sensor_data is None:
            rospy.logerr('In %s: line_sensor_data is None' % (self._action_name))

        # move forward
        if move_dir==1:
            # move apart
            msg.linear.x = v_const
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_t)

            while not rospy.is_shutdown():

                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                if array_data_sum==0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                array_data_mid = np.mean(np.nonzero(array_data)[1])+1
                array_data_mid_diff = array_data_mid-(array_data_len/2.0+0.5)

                v=v_const
                w=array_data_mid_diff*w_const

                if array_data_sum>array_data_len*(2.0/3.0):
                    v = 0
                    w = 0
                    success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # move backward
        if move_dir == 2:
            # move apart
            msg.linear.x = -v_const
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_t)

            while not rospy.is_shutdown():

                array_data = self.meg_sensor_data_b
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                if array_data_sum == 0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)

                v = -v_const
                w = array_data_mid_diff * w_const

                if array_data_sum > array_data_len * (2.0 / 3.0):
                    v = 0
                    w = 0
                    success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # turn left
        if move_dir == 3:
            # move apart
            msg.linear.x = v_const
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_t)

            msg.linear.x = 0
            msg.angular.z = 1.0 * w_rot
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_r)

            while not rospy.is_shutdown():

                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                array_data_mid_std = np.std(np.nonzero(array_data)[1])
                array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)

                if array_data_sum == 0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                v = 0
                w = 1.0 * w_rot

                if abs(array_data_mid_diff) < 1.0:
                    if array_data_mid_std<3.0: # eliminate unique case of sensing cross path and have [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1]
                        v = 0
                        w = 0
                        success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # turn right
        if move_dir == 4:
            # move apart
            msg.linear.x = v_const
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_t)

            # jump turn
            msg.linear.x = 0
            msg.angular.z = -1.0*w_rot
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_r)

            while not rospy.is_shutdown():

                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                array_data_mid_std = np.std(np.nonzero(array_data)[1])
                array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)

                if array_data_sum==0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                v = 0
                w = -1.0*w_rot

                if abs(array_data_mid_diff)<1.0:
                    if array_data_mid_std < 3.0:
                        v = 0
                        w = 0
                        success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # move forward without jump
        if move_dir == 5:

            while not rospy.is_shutdown():

                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                if array_data_sum == 0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)

                v = v_const
                w = array_data_mid_diff * w_const

                if array_data_sum > array_data_len * (2.0 / 3.0):
                    v = 0
                    w = 0
                    success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # move forward without jump, stop when no reading
        if move_dir == 7:
            while not rospy.is_shutdown():
                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                if array_data_sum == 0:
                    rospy.logwarn('In %s: line sensor has no reading' % (self._action_name))
                    v = 0
                    w = 0
                    success = True
                else:
                    array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                    array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)
                    v = v_const
                    w = array_data_mid_diff * w_const

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        # turn 360
        if move_dir == 9:
            # move apart
            msg.linear.x = v_const
            msg.angular.z = 0
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_t)

            # jump turn
            msg.linear.x = 0
            msg.angular.z = -1.0 * w_rot
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(jump_t_r)

            cross_num = 0

            while not rospy.is_shutdown():

                array_data = self.line_sensor_data
                array_data_len = array_data.shape[1]

                array_data_sum = np.sum(array_data)
                array_data_mid = np.mean(np.nonzero(array_data)[1]) + 1
                array_data_mid_std = np.std(np.nonzero(array_data)[1])
                array_data_mid_diff = array_data_mid - (array_data_len / 2.0 + 0.5)

                if array_data_sum == 0:
                    rospy.logwarn('In %s: meg sensor has no reading' % (self._action_name))
                    rospy.sleep(unit_t)
                    continue

                v = 0
                w = -1.0 * w_rot

                if abs(array_data_mid_diff) < 1.0:
                    if array_data_mid_std < 3.0:
                        if cross_num == 0:
                            cross_num=1
                            # jump turn
                            msg.linear.x = 0
                            msg.angular.z = -1.0 * w_rot
                            self.cmd_vel_pub.publish(msg)
                            rospy.sleep(jump_t_r)
                        elif cross_num == 1:
                            v = 0
                            w = 0
                            success = True

                msg.linear.x = v
                msg.angular.z = w
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(unit_t)

                if success == True:
                    break

        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)





if __name__ == '__main__':

    rospy.init_node('line_track_server_node')
    server = line_track_action()
    rospy.spin()
