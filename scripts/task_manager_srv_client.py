#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from patrol_robot.srv import *
import numpy as np
from std_msgs.msg import Float32MultiArray

if __name__ == "__main__":
    rospy.init_node('task_manager_srv_client_node', anonymous=True)
    print('starting client')
    rospy.wait_for_service('TaskList')
    print('service connected.')
    try:
        task_list_request = rospy.ServiceProxy('TaskList', TaskList)
        msg = Float32MultiArray()
        r = rospy.Rate(1)

        for n in range(10):
            task_list = np.zeros((20,10))
            task_list[n,0] = n

            task_list_flatten = task_list.reshape((1,-1))[0]
            task_list_flatten_list = task_list_flatten.tolist()

            msg.data = task_list_flatten_list

            print('send TaskList %s times' % (n))
            resp = task_list_request(msg)
            print('response is: %s' % (resp))

            r.sleep()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)