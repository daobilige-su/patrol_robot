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
import cv2
from scipy.ndimage import uniform_filter1d

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

        self.cam_img_np = None
        self.rate = rospy.Rate(1)

        self.img_v_por = [0, 0.3]
        self.negate_color = 1  # 0: white line, 1: black line
        self.lane_sensor_num = 7
        self.color_thr = 150
        self.blur_size = int(320/20)
        # self.uni_filt_size = int(320/20)

        self.verbose = 0

    def img_cb(self, msg):
        # get img
        try:
            self.cam_img_np = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        return

    def execute_img_to_line_sensor(self):

        if self.cam_img_np is None:
            rospy.logwarn('fws_cam_to_line_sensor: cam image not publishing.')
            return

        # rgb to grayscale
        cam_img_gray_np = cv2.cvtColor(self.cam_img_np, cv2.COLOR_BGR2GRAY)

        # revert color if it is a black line
        if self.negate_color:
            cam_img_gray_np = 255-cam_img_gray_np

        # get the top part of it
        cam_img_top_gray_np = cam_img_gray_np[int(self.img_v_por[0]*cam_img_gray_np.shape[0]):int(self.img_v_por[1]*cam_img_gray_np.shape[1])]

        # blur image
        cam_img_top_gray_np_blur = cv2.blur(cam_img_top_gray_np, (self.blur_size, self.blur_size))

        # take max along v axis
        cam_img_top_gray_np_mean = np.max(cam_img_top_gray_np_blur, axis=0)

        # # run moving average to filter out noise in color, because we are going to use the max of each chunk next
        # cam_img_top_gray_np_mean_ave = uniform_filter1d(cam_img_top_gray_np_mean, size=self.uni_filt_size)

        # reduce the array number to self.lane_sensor_num
        ls_idx = np.linspace(0, cam_img_top_gray_np_mean.shape, self.lane_sensor_num+1)
        lane_det_array = np.zeros((self.lane_sensor_num,))
        for n in range(self.lane_sensor_num):
            array_chunk = cam_img_top_gray_np_mean[int(ls_idx[n]):int(ls_idx[n+1])]
            lane_det_array[n] = np.max(array_chunk)

        if self.verbose:
            rospy.loginfo(lane_det_array)

        lane_det_array_bn = np.zeros((self.lane_sensor_num,))
        lane_det_array_bn[lane_det_array > self.color_thr] = 1

        # send the msg
        line_msg = Int8MultiArray()
        line_msg.data = lane_det_array_bn.astype(int)
        self.meg_sensor_pub.publish(line_msg)

        return


def main(args):
    rospy.init_node('fws_cam_to_line_node', anonymous=True)
    cam_to_line = CamToLine()
    try:
        while not rospy.is_shutdown():
            cam_to_line.execute_img_to_line_sensor()
            cam_to_line.rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
