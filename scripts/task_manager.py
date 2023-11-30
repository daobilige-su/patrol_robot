#! /usr/bin/env python
import numpy as np
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# goal message and the result message.
import move_base_msgs.msg

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from patrol_robot.srv import TaskList


class TaskManager:
    def __init__(self):
        self.sub = rospy.Subscriber("chatter", String, self.callback)
        self.task_list = np.zeros((20, 10))
        self.task_sleep_rate = rospy.Rate(10)
        self.task_list_srv = rospy.Service('TaskList', TaskList, self.update_task_list)
        print('service ready')

        #rospy.spin()

    def update_task_list(self, req):
        task_list = np.array(req.list.data).reshape((20, 10))
        print('received new task_list')
        print(task_list)

        return True


    def callback(self, data):
        print(data.data)


def main(args):
    rospy.init_node('task_manager_node', anonymous=True)
    tm = TaskManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)