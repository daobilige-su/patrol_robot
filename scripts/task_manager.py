#! /usr/bin/env python
import numpy
import numpy as np
import patrol_robot.msg
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# goal message and the result message.
import move_base_msgs.msg

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from patrol_robot.srv import TaskList
from tf import transformations


class TaskManager:
    def __init__(self):
        # self.sub = rospy.Subscriber("chatter", String, self.callback)
        self.task_list = np.zeros((20, 10))
        self.task_sleep_rate = rospy.Rate(10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # TaskList service to update self.task_list
        self.task_list_srv = rospy.Service('TaskList', TaskList, self.update_task_list)
        rospy.loginfo('TaskList service ready')

        # move_base and track_line clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        self.line_track_client = actionlib.SimpleActionClient('line_track', patrol_robot.msg.line_trackAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo('move_base_server connected.')
        self.line_track_client.wait_for_server()
        rospy.loginfo('line_track_server connected.')

        # ready to start
        rospy.logwarn('Going to Start after 3s ...')
        rospy.sleep(3)
        rospy.logwarn('Started!')

    # when client want to update task_list, cancel the current jobs and update the
    def update_task_list(self, req):
        task_list = np.array(req.list.data).reshape((20, 10))
        rospy.loginfo('received new task_list: ')
        rospy.loginfo(task_list)
        self.task_list = task_list.copy()  # copy a new duplicate, do not assign the reference

        self.move_base_client.cancel_all_goals()
        self.line_track_client.cancel_all_goals()

        # rospy.sleep(0.5)
        self.stop()

        return True

    def execute_task(self):
        task_list = self.task_list[0,:].copy()
        rospy.loginfo('Executing task: ')
        rospy.loginfo(task_list)

        self.task_list[0:10,:] = self.task_list[1:11,:].copy()
        self.task_list[10:11,:] = numpy.zeros((1,10)).copy()

        if task_list[0]==0:
            self.move_base_client.cancel_all_goals()
            self.line_track_client.cancel_all_goals()
            # publish all zero velocity cmd
            self.stop()
        elif task_list[0]==1: # move_base mode
            goal_pose = task_list[1:4].copy()
            self.move_base_action(goal_pose)
        elif task_list[0]==1: # track_line mode
            move_dir = task_list[0].copy()
            self.line_track_action(move_dir)
        else:
            rospy.logwarn('unknown task code.')

    def move_base_action(self, pose):
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        # goal.target_pose.pose.orientation.w = 1

        # tf.transformations.quaternion_from_euler(ai, aj, ak, axes='sxyz')
        # ai, aj, ak : Euler’s roll, pitch and yaw angles, axes : One of 24 axis sequences as string or encoded tuple
        # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
        # sequence: e.g. 'sxyz'
        # first character: rotations are applied to ‘s’tatic or ‘r’otating frame
        # remaining characters: successive rotation axis ‘x’, ‘y’, or ‘z’
        quat = transformations.quaternion_from_euler(0, 0, pose[2], 'rzyx')

        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        # Sends the goal to the action server.
        rospy.loginfo("move_base_client: sending goal [%s, %s, %s]" % (pose[0], pose[1], pose[2]))
        self.move_base_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.move_base_client.wait_for_result()
        rospy.loginfo("move_base_client: goal [%s, %s, %s] completed" % (pose[0], pose[1], pose[2]))

    # move_dir:
    # 1. move_base task: 0, stay still; 1, move forward; 2, move backward; 3, move left; 4, move right
    # 5, move forward no jump
    # 9, turn 180
    def line_track_action(self, move_dir):
        goal = patrol_robot.msg.line_trackGoal()
        goal.target_location = [move_dir]

        self.line_track_client.send_goal(goal)
        rospy.loginfo('line_track_client: sent new goal (%f)' % (goal.target_location[0]))

        self.line_track_client.wait_for_result()
        rospy.loginfo("line_track_client: goal completed")

    def stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        return True


def main(args):
    rospy.init_node('task_manager_node', anonymous=True)
    tm = TaskManager()
    try:
        while not rospy.is_shutdown():
            tm.execute_task()
            rospy.sleep(0.2)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)