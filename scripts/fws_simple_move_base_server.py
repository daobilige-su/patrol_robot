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

        cfgfile_path = rospy.get_param('param_yaml_file')
        with open(cfgfile_path, 'r') as stream:
            self.param = yaml.safe_load(stream)

        # cmd vel related params
        self.cmd_vel_x_p = self.param['simple_move_base']['cmd_vel_x_p']  # proportional gain w.r.t. trans err
        self.cmd_vel_theta_p = self.param['simple_move_base']['cmd_vel_theta_p']  # proportional gain w.r.t. rot err
        self.cmd_vel_x_max = self.param['simple_move_base']['cmd_vel_x_max']  # max trans vel
        self.cmd_vel_theta_max = self.param['simple_move_base']['cmd_vel_theta_max']  # max rot vel
        self.cmd_vel_x_min = self.param['simple_move_base']['cmd_vel_x_min']  # min trans vel
        self.cmd_vel_theta_min = self.param['simple_move_base']['cmd_vel_theta_min']  # min rot vel
        self.ctl_freq = self.param['simple_move_base']['ctl_freq']  # cmd_vel frequency

        # heading difference threshold to do pure rotation, if cur pose and goal pose's yaw difference is larger than
        # this, do only pure rotation, otherwise do rotation and translation together
        self.pure_rot_yaw_diff_thr = np.deg2rad(self.param['simple_move_base']['pure_rot_yaw_diff_thr'])

        # termination condition, trans and heading tolerance threshold
        self.pose_diff_dist_thr = self.param['simple_move_base']['pose_diff_dist_thr']
        self.pose_diff_yaw_thr = self.param['simple_move_base']['pose_diff_yaw_thr_deg'] * (math.pi / 180.0)

        # when end is precise localization: 0, number of continuous satisfaction of termination condition
        self.precise_mode_success_number = self.param['simple_move_base']['precise_mode_success_number']

        # in pass through mode, terminate when trans tolerance error is <
        # self.pass_through_trans_error_factor*self.pose_diff_dist_thr (dont care about heading)
        self.pass_through_trans_error_factor = 3.0

        self.verbose = self.param['simple_move_base']['verbose']

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(self.ctl_freq)
        success = False

        if self.verbose:
            rospy.loginfo('%s: Executing, obtained goal pose of (%f, %f, %f, %f, %f, %f, %f)' % (
                self._action_name, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z,
                goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))

        # the 3 termination mode is set based on goal pose's z coord, 11.0: precise mode, 12.0: pass through mode
        end_mode = 0
        if goal.target_pose.pose.position.z == 11.0:
            end_mode = 1
        elif goal.target_pose.pose.position.z == 12.0:
            end_mode = 2

        pose_tar_trans = np.array([[goal.target_pose.pose.position.x], [goal.target_pose.pose.position.y], [goal.target_pose.pose.position.z]])
        pose_tar_quat = np.array([[goal.target_pose.pose.orientation.x], [goal.target_pose.pose.orientation.y], [goal.target_pose.pose.orientation.z], [goal.target_pose.pose.orientation.w]])
        pose_tar_ypr = quat2ypr(pose_tar_quat)
        pose_tar_trans_ypr = np.block([[pose_tar_trans],[pose_tar_ypr]])
        pose_tar_M = transform_matrix_from_trans_ypr(pose_tar_trans_ypr)

        # start executing the action
        cmd_vel_x_diff = 0.0
        cmd_vel_theta_diff = 0.0
        cmd_vel_x = 0.0
        cmd_vel_theta= 0.0
        cmd_vel_msg = Twist()

        # when end is precise localization: 1, number of continuous satisfaction of termination condition
        end_mode_1_succ_num = 0
        while success!=True:

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

            if self.verbose:
                rospy.loginfo('pose_diff_trans_ypr: (%f, %f, %f)' % (pose_diff_trans_ypr[0,0],pose_diff_trans_ypr[1,0],pose_diff_trans_ypr[3,0]))

            pose_diff_M_2d_line_yaw = np.arctan2(pose_diff_trans_ypr[1,0],pose_diff_trans_ypr[0,0])
            pose_diff_M_2d_line_dist = np.sqrt(pose_diff_trans_ypr[0,0]**2 + pose_diff_trans_ypr[1,0]**2)

            # compute error in x, y and theta, based on which the cmd_vel is send by multiplying a proportional gain
            # if pose heading difference is very large, do pure rotation first, otherwise do translation and rotation together
            if abs(pose_diff_trans_ypr[3, 0]) > self.pure_rot_yaw_diff_thr:
                cmd_vel_x_diff = 0.0
                cmd_vel_y_diff = 0.0
                cmd_vel_theta_diff = np.sign(pose_diff_trans_ypr[3, 0]) * self.pure_rot_yaw_diff_thr  # 0.35
            else:
                cmd_vel_x_diff = pose_diff_M_2d_line_dist * np.cos(pose_diff_M_2d_line_yaw)
                cmd_vel_y_diff = pose_diff_M_2d_line_dist * np.sin(pose_diff_M_2d_line_yaw)
                cmd_vel_theta_diff = pose_diff_trans_ypr[3, 0]

            # (0) Normal ending mode, when termination condition is met, set cmd_vel to 0, and return.
            if end_mode==0:
                if (abs(pose_diff_trans_ypr[3, 0]) < self.pose_diff_yaw_thr) and (pose_diff_M_2d_line_dist < self.pose_diff_dist_thr):
                    cmd_vel_x_diff = 0.0
                    cmd_vel_y_diff = 0.0
                    cmd_vel_theta_diff = 0.0
                    success = True
            # (1) Precise mode, when termination condition is continuously satisfied for self.precise_mode_success_number
            # times, terminate.
            elif end_mode==1:
                if (abs(pose_diff_trans_ypr[3, 0]) < self.pose_diff_yaw_thr) and (pose_diff_M_2d_line_dist < self.pose_diff_dist_thr):
                    end_mode_1_succ_num = end_mode_1_succ_num+1
                    if end_mode_1_succ_num == self.precise_mode_success_number:
                        cmd_vel_x_diff = 0.0
                        cmd_vel_y_diff = 0.0
                        cmd_vel_theta_diff = 0.0
                        success = True
                        end_mode_1_succ_num = 0
                else:
                    end_mode_1_succ_num = 0
            # (2) Pass through mode, terminate when trans tolerance error is <
            # self.pass_through_trans_error_factor*self.pose_diff_dist_thr (dont care about heading)
            # terminate without setting cmd_vel to zero
            elif end_mode==2:
                if pose_diff_M_2d_line_dist < self.pass_through_trans_error_factor*self.pose_diff_dist_thr:
                    success = True

            cmd_vel_x = cmd_vel_x_diff * self.cmd_vel_x_p
            cmd_vel_y = cmd_vel_y_diff * self.cmd_vel_x_p
            cmd_vel_theta = cmd_vel_theta_diff * self.cmd_vel_theta_p

            # max velocity constraints
            # trans max velo
            if np.sqrt(cmd_vel_x**2 + cmd_vel_y**2) > self.cmd_vel_x_max:
                cmd_vel_x = cmd_vel_x * (self.cmd_vel_x_max / np.sqrt(cmd_vel_x ** 2 + cmd_vel_y ** 2))
                cmd_vel_y = cmd_vel_y * (self.cmd_vel_x_max / np.sqrt(cmd_vel_x ** 2 + cmd_vel_y ** 2))
            # rot max velo
            if cmd_vel_theta>self.cmd_vel_theta_max:
                cmd_vel_theta = self.cmd_vel_theta_max
            elif cmd_vel_theta<(-1.0*self.cmd_vel_theta_max):
                cmd_vel_theta = -1.0*self.cmd_vel_theta_max

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

            # publish the feedback
            # self._feedback.base_position.header = TODO
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()  # /clock needs to publishing (explicitly publish clock if /use_sim_time is set)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('simple_move_base_server')
    server = simple_move_base_action()
    rospy.spin()
