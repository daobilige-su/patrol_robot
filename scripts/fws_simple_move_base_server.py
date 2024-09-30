#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import move_base_msgs.msg
import tf
from geometry_msgs.msg import Twist
import numpy as np
import math

from transform_tools import *

import yaml

cfgfile_path = rospy.get_param('param_yaml_file')
with open(cfgfile_path, 'r') as stream:
    cfg_data = yaml.safe_load(stream)

# pose_diff_dist_thr = 0.1
pose_diff_dist_thr = cfg_data['simple_move_base']['pose_diff_dist_thr']
# pose_diff_yaw_thr = 5.0*(math.pi/180.0)
pose_diff_yaw_thr = cfg_data['simple_move_base']['pose_diff_yaw_thr_deg']*(math.pi/180.0)
# pose_diff_2d_line_yaw_thr = 10.0*(math.pi/180.0)
pose_diff_2d_line_yaw_thr = cfg_data['simple_move_base']['pose_diff_2d_line_yaw_thr_deg']*(math.pi/180.0)

# cmd_vel_x_p = 0.75
cmd_vel_x_p = cfg_data['simple_move_base']['cmd_vel_x_p']
# cmd_vel_theta_p = 0.75
cmd_vel_theta_p = cfg_data['simple_move_base']['cmd_vel_theta_p']
# cmd_vel_x_max = 0.25
cmd_vel_x_max = cfg_data['simple_move_base']['cmd_vel_x_max']
# cmd_vel_theta_max = 0.25
cmd_vel_theta_max = cfg_data['simple_move_base']['cmd_vel_theta_max']
# cmd_vel_x_min = 0.0
cmd_vel_x_min = cfg_data['simple_move_base']['cmd_vel_x_min']
# cmd_vel_theta_min = 0.0
cmd_vel_theta_min = cfg_data['simple_move_base']['cmd_vel_theta_min']


class simple_move_base_action(object):
    # create messages that are used to publish feedback/result
    _feedback = move_base_msgs.msg.MoveBaseFeedback() # geometry_msgs/PoseStamped base_position
    _result = move_base_msgs.msg.MoveBaseResult() # empty

    def __init__(self):
        self._action_name = 'simple_move_base'
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(5)
        success = False

        # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)

        # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (
        # self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        rospy.loginfo('%s: Executing, obtained goal pose of (%f, %f, %f, %f, %f, %f, %f)' % (
            self._action_name, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z,
            goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))

        pose_tar_trans = np.array([[goal.target_pose.pose.position.x], [goal.target_pose.pose.position.y], [goal.target_pose.pose.position.z]])
        pose_tar_quat = np.array([[goal.target_pose.pose.orientation.x], [goal.target_pose.pose.orientation.y], [goal.target_pose.pose.orientation.z], [goal.target_pose.pose.orientation.w]])
        pose_tar_ypr = quat2ypr(pose_tar_quat)
        pose_tar_trans_ypr = np.block([[pose_tar_trans],[pose_tar_ypr]])
        pose_tar_M = transform_matrix_from_trans_ypr(pose_tar_trans_ypr)

        # start executing the action
        n = 0
        mode = -1 # -1: uninitialized, 1: rotate to face target direction, 2: drive to target, 3: rotate to target pose direction, 4: done
        cmd_vel_x_diff = 0.0
        cmd_vel_theta_diff = 0.0
        cmd_vel_x = 0.0
        cmd_vel_theta= 0.0
        cmd_vel_msg = Twist()

        mode = 1
        while success!=True:
            n = n + 1

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                # success = False
                break

            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            pose_cur_trans = np.array([[trans[0]], [trans[1]], [trans[2]]])
            pose_cur_quat = np.array([[rot[0]], [rot[1]], [rot[2]], [rot[3]]]) # rot (x, y, z, w)
            pose_cur_ypr = quat2ypr(pose_cur_quat)
            pose_cur_trans_ypr = np.block([[pose_cur_trans], [pose_cur_ypr]])
            pose_cur_M = transform_matrix_from_trans_ypr(pose_cur_trans_ypr)

            pose_diff_M = np.linalg.lstsq(pose_cur_M,pose_tar_M, rcond=None)[0] # pose_cur_M\pose_tar_M: target pose in current pose's coordinate
            pose_diff_trans_ypr = transform_matrix_to_pose_trans_ypr(pose_diff_M)

            rospy.loginfo('pose_diff_trans_ypr: (%f, %f, %f)' % (pose_diff_trans_ypr[0,0],pose_diff_trans_ypr[1,0],pose_diff_trans_ypr[3,0]))

            pose_diff_M_2d_line_yaw = np.arctan2(pose_diff_trans_ypr[1,0],pose_diff_trans_ypr[0,0])
            pose_diff_M_2d_line_dist = np.sqrt(pose_diff_trans_ypr[0,0]**2 + pose_diff_trans_ypr[1,0]**2)

            # compute error in x, y and theta
            if abs(pose_diff_trans_ypr[3, 0]) > pose_diff_yaw_thr:
                cmd_vel_x_diff = 0.0
                cmd_vel_y_diff = 0.0
                cmd_vel_theta_diff = np.sign(pose_diff_trans_ypr[3, 0]) * 0.35
            else:
                cmd_vel_x_diff = pose_diff_M_2d_line_dist * np.cos(pose_diff_M_2d_line_yaw)
                cmd_vel_y_diff = pose_diff_M_2d_line_dist * np.sin(pose_diff_M_2d_line_yaw)
                cmd_vel_theta_diff = pose_diff_trans_ypr[3, 0]

            if (abs(pose_diff_trans_ypr[3, 0]) < pose_diff_yaw_thr) and (pose_diff_M_2d_line_dist < pose_diff_dist_thr):
                cmd_vel_x_diff = 0.0
                cmd_vel_y_diff = 0.0
                cmd_vel_theta_diff = 0.0
                success = True

            cmd_vel_x = cmd_vel_x_diff * cmd_vel_x_p
            cmd_vel_y = cmd_vel_y_diff * cmd_vel_x_p
            cmd_vel_theta = cmd_vel_theta_diff * cmd_vel_theta_p

            # max velocity constraints
            if np.sqrt(cmd_vel_x**2 + cmd_vel_y**2) > cmd_vel_x_max:
                cmd_vel_x = cmd_vel_x * (cmd_vel_x_max / np.sqrt(cmd_vel_x ** 2 + cmd_vel_y ** 2))
                cmd_vel_y = cmd_vel_y * (cmd_vel_x_max / np.sqrt(cmd_vel_x ** 2 + cmd_vel_y ** 2))

            if cmd_vel_theta>cmd_vel_theta_max:
                cmd_vel_theta = cmd_vel_theta_max
            elif cmd_vel_theta<(-1.0*cmd_vel_theta_max):
                cmd_vel_theta = -1.0*cmd_vel_theta_max

            # min velocity constraints # TODO
            # if mode == 2:
            #     if (cmd_vel_x > 0) and (cmd_vel_x < cmd_vel_x_min):
            #         cmd_vel_x = cmd_vel_x_min
            #     elif (cmd_vel_x < 0) and (cmd_vel_x > (-1.0 * cmd_vel_x_min)):
            #         cmd_vel_x = -1.0 * cmd_vel_x_min
            # elif (mode == 1) or (mode == 3):
            #     if (cmd_vel_theta > 0) and (cmd_vel_theta < cmd_vel_theta_min):
            #         cmd_vel_theta = cmd_vel_theta_min
            #     elif (cmd_vel_theta < 0) and (cmd_vel_theta > (-1.0 * cmd_vel_theta_min)):
            #         cmd_vel_theta = -1.0 * cmd_vel_theta_min

            cmd_vel_msg.linear.x = cmd_vel_x
            cmd_vel_msg.linear.y = cmd_vel_y
            cmd_vel_msg.angular.z = cmd_vel_theta

            # publish "cmd_vel"
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i - 1])
            # publish the feedback
            # self._feedback.base_position.header = TODO
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()# /clock needs to publishing (explicitely publish clock if /use_sim_time is set)

        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('simple_move_base_server')
    server = simple_move_base_action()
    rospy.spin()