#! /usr/bin/env python

import rospy, rospkg
import sys
import tf
from std_msgs.msg import Float64MultiArray
from gps_common.msg import GPSFix
import numpy as np
from geo_coord_transform import *
from transform_tools import *
import yaml

gps_src = 0  # 0: sim, 1: fdi

class gps_localizer:
    def __init__(self):
        # locate ros pkg
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('mowerbot') + '/'
        # load key params
        # params_filename = rospy.get_param('param_file')  # self.pkg_path + 'cfg/' + 'param.yaml'
        params_filename = self.pkg_path + 'param/' + 'mowerbot_params.yaml'
        with open(params_filename, 'r') as file:
            self.param = yaml.safe_load(file)

        self.send_rate = rospy.Rate(10)  # send with 10 hz
        self.br = tf.TransformBroadcaster()

        self.lla_ori = np.array(self.param['map']['ref_lla'])
        self.sim_gps_sub = rospy.Subscriber('dgps_floatarray', Float64MultiArray, self.sim_gps_cb)
        self.fdi_gps_sub = rospy.Subscriber('/gnss_dual_ant/fix', GPSFix, self.fdi_gps_cb)

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    def send_tf(self):
        self.tf_broadcast([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
                          'laser_link', 'base_link')

    def sim_gps_cb(self, msg):
        # if gps_src == 0:
        if True:
            data = np.array(msg.data)
            lla = data[0:3]
            track = data[3]
            theta = track+np.pi/2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])

            quat = ypr2quat(np.array([theta, 0, 0]))

            self.tf_broadcast(enu, quat, 'base_link', 'map')

    def fdi_gps_cb(self, msg):
        # if gps_src == 1:
        if True:
            # lla = np.array([np.deg2rad(msg.latitude), np.deg2rad(msg.longitude), msg.altitude])
            lla = np.array([msg.latitude, msg.longitude, msg.altitude])
            track = -np.deg2rad(msg.track) # the direction of gps angle is opposite of Z axis up, maybe it is NED, not ENU
            theta = track + np.pi / 2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])

            quat = ypr2quat(np.array([theta, 0, 0]))

            self.tf_broadcast(enu, quat, 'base_link', 'map')


def main(args):
    rospy.init_node('gps_loc_node', anonymous=True)
    gl = gps_localizer()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
