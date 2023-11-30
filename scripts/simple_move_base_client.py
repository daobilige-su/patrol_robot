#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import move_base_msgs.msg
from geometry_msgs.msg import PoseStamped

client = actionlib.SimpleActionClient('simple_move_base', move_base_msgs.msg.MoveBaseAction)

def sub_callback(pose_msg):
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose = pose_msg

    client.send_goal(goal)
    rospy.loginfo('simple_move_base_client: sent new goal (%f, %f, %f, %f, %f, %f, %f)' % (
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z,
            goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))

    client.wait_for_result()
    rospy.loginfo("simple_move_base_client: goal completed")

def simple_move_base_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    # client = actionlib.SimpleActionClient('simple_move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # goal = move_base_msgs.msg.MoveBaseGoal()

    rospy.Subscriber("move_base_simple/goal", PoseStamped, sub_callback)

    # goal.target_pose.pose.orientation.w = 1
    #
    # for i in range(10):
    #     goal.target_pose.pose.position.x = i
    #
    #     # Sends the goal to the action server.
    #     rospy.loginfo("simple_move_base_client: sending goal %i" % i)
    #     client.send_goal(goal)
    #
    #     # Waits for the server to finish performing the action.
    #     client.wait_for_result()
    #     rospy.loginfo("simple_move_base_client: goal %i completed" % i)
    #
    #     # Prints out the result of executing the action
    #     # return client.get_result()  # A FibonacciResult
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('simple_move_base_client')
        result = simple_move_base_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")