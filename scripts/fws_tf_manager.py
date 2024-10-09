#! /usr/bin/env python

import rospy
import sys
import tf
import yaml
import numpy as np
from transform_tools import *


class TFManager:
    def __init__(self):
        # load params
        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        # load tf from param file
        trans_ypr_laserlink_in_baselink = np.array(self.param['tf']['tran_ypr_laserlink_in_baselink'])
        quat_laserlink_in_baselink = ypr2quat(trans_ypr_laserlink_in_baselink[3:])
        self.trans_quat_laserlink_in_baselink = np.array(
            [trans_ypr_laserlink_in_baselink[0], trans_ypr_laserlink_in_baselink[1], trans_ypr_laserlink_in_baselink[2],
             quat_laserlink_in_baselink[0, 0], quat_laserlink_in_baselink[1, 0], quat_laserlink_in_baselink[2, 0], quat_laserlink_in_baselink[3, 0]])

        # define other terms
        self.send_rate = rospy.Rate(10)  # send with 10 hz
        self.br = tf.TransformBroadcaster()

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    def send_tf(self):
        self.tf_broadcast(self.trans_quat_laserlink_in_baselink[:3], self.trans_quat_laserlink_in_baselink[3:],
                          'laser_link', 'base_link')


def main(args):
    rospy.init_node('tf_manager_node', anonymous=True)
    tfm = TFManager()
    try:
        while not rospy.is_shutdown():
            tfm.send_tf()
            tfm.send_rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
