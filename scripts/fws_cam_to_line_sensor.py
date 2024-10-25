#! /usr/bin/env python

import rospy
import sys
import tf
import yaml
import numpy as np
from transform_tools import *
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Int8MultiArray

from cv_bridge import CvBridge, CvBridgeError

class CamToLine:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot') + '/'

        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        self.image_sub = rospy.Subscriber("/front_rgb_cam/image_raw", Image, self.img_cb)
        self.meg_sensor_pub = rospy.Publisher("meg_sensor", Int8MultiArray, queue_size=1)

        self.bridge = CvBridge()

        self.img_v_por = [0, 0.3]

    def img_cb(self, msg):
        # get img
        try:
            cam_img_np = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        return


def main(args):
    rospy.init_node('fws_cam_to_line_node', anonymous=True)
    cam_to_line = CamToLine()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
