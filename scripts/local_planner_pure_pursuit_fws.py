#! /usr/bin/env python

import numpy as np
import numpy.linalg
import math
# from __future__ import print_function

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
import rospy
import sys
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32MultiArray
from tf import transformations
import tf
from transform_tools import *
from std_srvs.srv import SetBool, SetBoolResponse

from actionlib_msgs.msg import GoalID
import yaml


class PurePursuitPlannerFws:
    def __init__(self):
        # param
        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        self.controller_freq = self.param['pure_pursuit_planner_fws']['controller_freq']  # freq of sending ctrl cmd
        self.look_ahead_dist = self.param['pure_pursuit_planner_fws']['look_ahead_dist']  # look head dist of pure pursuit
        self.v_dist_max = self.param['pure_pursuit_planner_fws']['v_dist_max']  # if dist error is larger than this, use v_max
        self.v_max = self.param['pure_pursuit_planner_fws']['v_max']  # max linear velocity
        self.w_heading_max = np.deg2rad(self.param['pure_pursuit_planner_fws']['w_heading_max'])  # if heading error is larger than this, use w_max
        self.w_max = np.deg2rad(self.param['pure_pursuit_planner_fws']['w_max'])  # max angular velocity

        self.nav_tol_dist = self.param['pure_pursuit_planner_fws']['nav_tol_dist']  # tolerance dist of goal pose
        self.nav_tol_heading = np.deg2rad(self.param['pure_pursuit_planner_fws']['nav_tol_heading'])  # tolerance heading of goal pose

        # if all poses in path has the same heading with the last pose, since by default only the last pose of path has
        # the same heading with the goal pose with rest of them have heading values of zeros.
        self.end_heading_on = self.param['pure_pursuit_planner_fws']['end_heading_on']

        self.verbose = self.param['pure_pursuit_planner_fws']['verbose']  # verbose flag

        # var to store the current global plan
        self.global_plan = None

        # tf listener
        self.tf_listener = tf.TransformListener()

        # publishers & subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        # self.global_plan_sub = rospy.Subscriber("/global_plan", Path, self.update_global_plan)
        self.global_plan_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.update_global_plan)
        # after the goal is reached, cancel the current move_base action, or its own local planner might still run
        self.move_base_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

        # stop service
        self.stop_srv = rospy.Service('local_planner_pure_pursuit_stop', SetBool, self.stop_cb)

        return

    def update_global_plan(self, global_path_msg):
        pose_num = len(global_path_msg.poses)
        global_plan = np.zeros((pose_num, 6))

        # rospy.loginfo('received new global plan')
        # rospy.loginfo('pose_num = %d' % pose_num)
        for n in range(pose_num):
            # print('%d / %d' % (n, pose_num))
            pose_trans_quat = np.array([global_path_msg.poses[n].pose.position.x,
                                        global_path_msg.poses[n].pose.position.y,
                                        global_path_msg.poses[n].pose.position.z,
                                        global_path_msg.poses[n].pose.orientation.x,
                                        global_path_msg.poses[n].pose.orientation.y,
                                        global_path_msg.poses[n].pose.orientation.z,
                                        global_path_msg.poses[n].pose.orientation.w])

            trans = pose_trans_quat[0:3]
            quat = pose_trans_quat[3:7]
            ypr = quat2ypr(quat).reshape((-1,))
            pose_trans_ypr = np.block([trans, ypr])
            global_plan[n, :] = pose_trans_ypr.copy()

        # use the heading of the last pose in the path
        # while in global_planner's orientation_filter=None(=0) mode, only the heading the of last pose of the path is
        # the heading of the goal pose, the rest of them is 0. (http://wiki.ros.org/global_planner?distro=noetic)
        # so, we can set the heading of the all pose in the path to be the heading of the goal pose.
        # but in practice even the last pose of the path is occasionally with the heading zero. strange!
        # Maybe use orientation_filter=Interpolate=2 (Orientations are a linear blend of start and goal pose) ?
        if self.end_heading_on:
            global_plan[:, 3] = global_plan[-1, 3]

        self.global_plan = global_plan.copy()

    @staticmethod
    def wraptopi(x):
        pi = np.pi
        x = x - np.floor(x / (2 * pi)) * 2 * pi
        if np.isscalar(x):
            if x>=pi:
                x = x -2 * pi
        else:
            x[x >= pi] = x[x >= pi] - 2 * pi
        return x

    def compute_plan(self):
        # skip if the global plan is None
        if self.global_plan is None:
            if self.verbose:
                rospy.loginfo('local_planner_pure_pursuit_fws_node: no global plan')
            return

        # listen to the latest tf for base_link in map
        try:
            # trans: [x, y, z], rot: [qx, qy, qz, qw]
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # compute pose and path in 2d
        ypr = quat2ypr(np.array(rot)).reshape((-1,))
        rob_pose = np.array([trans[0], trans[1], ypr[0]])

        path = self.global_plan[:, [0, 1, 3]]
        path_pose_num = path.shape[0]
        if path_pose_num == 0:
            if self.verbose:
                rospy.logwarn('local_planner_pure_pursuit_fws_node: path is empty, returning.')
            return

        # check if termination condition is satisfied
        # print(np.linalg.norm(rob_pose[0:2]-path[-1,0:2]))
        # print(np.unwrap(np.array([rob_pose[2]-path[-1,2]]))[0])
        # print(abs(np.unwrap(np.array([rob_pose[2]-path[-1,2]]))[0]-np.pi))
        # print(path)
        if np.linalg.norm(rob_pose[0:2]-path[-1,0:2])<self.nav_tol_dist and abs(self.wraptopi(rob_pose[2]-path[-1,2]))<self.nav_tol_heading:
            # send cmd_vel
            cmd_vel_msg = Twist()
            self.cmd_vel_pub.publish(cmd_vel_msg)
            if self.verbose:
                rospy.logwarn('local_planner_pure_pursuit_fws: goal pose accomplished')

            # cancel current move_base action
            cancel_msg = GoalID()
            self.move_base_cancel_pub.publish(cancel_msg)
            if self.verbose:
                rospy.logwarn('move_base cancel msg sent')

            self.global_plan = None
            return

        # nearest path pt
        dist = np.linalg.norm(np.tile(rob_pose[0:2], [path_pose_num, 1]) - path[:, 0:2], axis=1)
        nearest_pt_idx = np.argmin(dist)

        # look ahead pt
        look_ahead_pt_idx = nearest_pt_idx
        while dist[look_ahead_pt_idx]<self.look_ahead_dist:
            look_ahead_pt_idx = look_ahead_pt_idx+1
            if look_ahead_pt_idx>(path_pose_num-1):
                break
        look_ahead_pt_idx = look_ahead_pt_idx-1
        look_ahead_pt = path[look_ahead_pt_idx, :]

        # print('look_ahead_pt[2] = %f' % look_ahead_pt[2])

        # relative pose
        rob_pose_3d_T = transform_trans_ypr_to_matrix(np.array([rob_pose[0], rob_pose[1], 0, rob_pose[2], 0, 0]))
        look_ahead_pt_3d_T = transform_trans_ypr_to_matrix(np.array([look_ahead_pt[0], look_ahead_pt[1], 0, look_ahead_pt[2], 0, 0]))
        relative_pose_3d_T = np.linalg.solve(rob_pose_3d_T, look_ahead_pt_3d_T)

        relative_pose_3d_trans_ypr = transform_matrix_to_trans_ypr(relative_pose_3d_T).reshape((-1,))
        relative_pose = np.array([relative_pose_3d_trans_ypr[0], relative_pose_3d_trans_ypr[1], relative_pose_3d_trans_ypr[3]])

        # print('look_ahead_pt_idx: %f, relative_pose: [%f, %f, %f]' % (look_ahead_pt_idx, relative_pose[0], relative_pose[1], relative_pose[2]))

        # compute v and w
        rel_dist = np.linalg.norm(relative_pose[0:2])
        rel_theta = math.atan2(relative_pose[1], relative_pose[0])

        if rel_dist>self.v_dist_max:
            v = self.v_max
        else:
            v = self.v_max*(rel_dist/self.v_dist_max)
        v_x = v * np.cos(rel_theta)
        v_y = v * np.sin(rel_theta)
        # print('rel_dist:, %f, rel_theta: %f, vx: %f, vy: %f' % (rel_dist, rel_theta, v_x, v_y))

        rel_heading = relative_pose[2]
        if abs(rel_heading)>self.w_heading_max:
            w = self.w_max * np.sign(rel_heading)
        else:
            w = self.w_max * (rel_heading/self.w_heading_max)
        # w=0

        # send cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = v_x
        cmd_vel_msg.linear.y = v_y
        cmd_vel_msg.angular.z = w

        self.cmd_vel_pub.publish(cmd_vel_msg)

        return True

    def stop_cb(self, req):
        if self.verbose:
            rospy.logwarn('local_planner_pure_pursuit_fws: stop srv called')

        # send cmd_vel
        cmd_vel_msg = Twist()
        self.cmd_vel_pub.publish(cmd_vel_msg)

        # cancel current move_base action
        cancel_msg = GoalID()
        self.move_base_cancel_pub.publish(cancel_msg)
        if self.verbose:
            rospy.logwarn('move_base cancel msg sent')

        # cancel current global plan
        self.global_plan = None

        # return response
        res = SetBoolResponse()
        res.success = True
        res.message = 'stop srv called, vel is set to zero, move_base cancel msg sent.'

        return res


def main(args):
    rospy.init_node('local_planner_pure_pursuit_fws_node', anonymous=True)
    planner = PurePursuitPlannerFws()
    rate = rospy.Rate(planner.controller_freq)
    try:
        while not rospy.is_shutdown():
            planner.compute_plan()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)