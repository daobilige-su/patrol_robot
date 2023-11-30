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

marker_pub = rospy.Publisher("fixed_traj_markers", MarkerArray, queue_size=5)

# https://www.cnblogs.com/mxleader/p/17285004.html
client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

traj_pts = np.array([[0.0,0,0],
                     [1.0,0,0],
                     [1.0,-0.4,0],
                     [4.5,-0.4,0],
                     [4.5,-6.4,0],
                     [3.5,-6.4,0],
                     [4.5,-6.4,0],
                     [4.5,-14.4,0]])
traj_pts_num = traj_pts.shape[0]

def plot_traj():
    marker_array = MarkerArray()
    marker_array.markers = []

    # traj_pts
    pt_marker = Marker()
    pt_marker.header.frame_id = "map"
    pt_marker.ns = "fixed_traj_" + "pt"
    pt_marker.id = 0
    pt_marker.type = Marker.CUBE_LIST
    pt_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1.0
    pt_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    pt_marker.color.r, pt_marker.color.g, pt_marker.color.b = (1.0, 1.0, 0)
    pt_marker.color.a = 1.0
    pt_marker.scale.x, pt_marker.scale.y, pt_marker.scale.z = (0.3, 0.3, 0.3)

    pt_marker.points = []
    # pt_marker.colors = []
    for i in range(traj_pts_num):
        pt = Point()
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = 0
        pt_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # pt_marker.colors.append(color)

    marker_array.markers.append(pt_marker)

    # traj line
    line_marker = Marker()
    line_marker.header.frame_id = "map"
    line_marker.ns = "fixed_traj_" + "line"
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1
    line_marker.pose = pose
    line_marker.color.r, line_marker.color.g, line_marker.color.b = (1.0, 0.0, 0.0)
    line_marker.color.a = 0.5
    line_marker.scale.x, line_marker.scale.y, line_marker.scale.z = (0.1, 0.1, 0.1)

    line_marker.points = []
    for i in range(traj_pts_num):
        pt = Point()
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = 0
        line_marker.points.append(pt)

    marker_array.markers.append(line_marker)

    # publish marker_array
    global marker_pub
    marker_pub.publish(marker_array)


def move_base_client():

    # Waits until the action server has started up and started
    client.wait_for_server()
    rospy.loginfo('move_base_server connected.')

    rospy.loginfo('Going to Start after 5s ...')
    rospy.sleep(2)
    plot_traj()
    rospy.sleep(3)
    rospy.loginfo('Started!')

    # Creates a goal to send to the action server.
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.w = 1


    for i in range(traj_pts_num):
        plot_traj()
        goal.target_pose.pose.position.x = traj_pts[i, 0]
        goal.target_pose.pose.position.y = traj_pts[i, 1]
        goal.target_pose.pose.orientation.z = traj_pts[i, 2]

        # Sends the goal to the action server.
        rospy.loginfo("move_base_client: sending goal %i" % i)
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        rospy.loginfo("simple_move_base_client: goal %i completed" % i)
    #
        # Prints out the result of executing the action
        # return client.get_result()  # A FibonacciResult
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fixed_traj_nav')
        plot_traj()
        move_base_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")