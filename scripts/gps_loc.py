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

from std_srvs.srv import SetBool, SetBoolResponse

class gps_localizer:
    def __init__(self):
        # locate ros pkg
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot') + '/'
        # load key params
        cfgfile_path = rospy.get_param('param_yaml_file')
        with open(cfgfile_path, 'r') as stream:
            self.param = yaml.safe_load(stream)

        self.send_rate = rospy.Rate(10)  # send with 10 hz
        self.br = tf.TransformBroadcaster()

        self.lla_ori = np.array(self.param['gps']['ref_lla'])
        self.sim_gps_sub = rospy.Subscriber('dgps_floatarray', Float64MultiArray, self.sim_gps_cb)
        self.fdi_gps_sub = rospy.Subscriber('/gnss_dual_ant/fix', GPSFix, self.fdi_gps_cb)

        # TaskList service to update self.task_list
        self.gps_loc_on = (self.param['loc']['loc_src']==1)
        rospy.logwarn('gps_loc status: ' + str(self.gps_loc_on))
        self.gps_loc_on_srv = rospy.Service('GpsLocOn', SetBool, self.update_gps_loc_on)
        rospy.logwarn('GpsLocOn service ready')

        # self.gps_src = self.param['gps']['src']  # 0: sim, 1: fdi
        self.tran_ypr_map_in_enu = np.array(self.param['tf']['tran_ypr_map_in_enu'])
        self.T_map_in_enu = transform_trans_ypr_to_matrix(self.tran_ypr_map_in_enu)
        self.tran_ypr_baselink_in_gps = np.array(self.param['tf']['tran_ypr_baselink_in_gps'])
        self.T_baselink_in_gps = transform_trans_ypr_to_matrix(self.tran_ypr_baselink_in_gps)

        self.tf_listener = tf.TransformListener()

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    def update_gps_loc_on(self, req):
        if req.data:
            self.gps_loc_on = 1
            rospy.logwarn('gps_loc status is updated to: True')
            res = SetBoolResponse()
            res.success = True
            res.message = 'gps_loc status is updated to: True'
        else:
            self.gps_loc_on = 0
            rospy.logwarn('gps_loc status is updated to: False')
            res = SetBoolResponse()
            res.success = False
            res.message = 'gps_loc status is updated to: False'
        return res

    def sim_gps_cb(self, msg):
        if self.gps_loc_on:
            data = np.array(msg.data)
            lla = data[0:3]
            track = data[3]
            theta = track+np.pi/2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            # (1) get base_link in map
            #  x, y, z coords for East, North, Up frame in map
            trans_gps_in_enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])
            # Quaternion for East, North, Up frame in map
            ypr_gps_in_enu = np.array([theta, 0, 0])

            T_gps_in_enu = transform_trans_ypr_to_matrix(np.array([trans_gps_in_enu[0], trans_gps_in_enu[1], 0.0,
                                                                   ypr_gps_in_enu[0], ypr_gps_in_enu[1], ypr_gps_in_enu[2]]))

            T_baselink_in_enu = T_gps_in_enu @ self.T_baselink_in_gps

            # T_baselink_in_map = T_map_in_enu\T_baselink_in_enu: base_link in map's coordinate (left divide)
            T_baselink_in_map = np.linalg.lstsq(self.T_map_in_enu, T_baselink_in_enu, rcond=None)[0]

            # (2) get base_link in odom
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return
            trans_baselink_in_odom = np.array([[trans[0]], [trans[1]], [trans[2]]])
            quat_baselink_in_odom = np.array([[rot[0]], [rot[1]], [rot[2]], [rot[3]]])  # rot (x, y, z, w)

            T_baselink_in_odom = transform_trans_quat_to_matrix(
                np.array([trans_baselink_in_odom[0, 0], trans_baselink_in_odom[1, 0], trans_baselink_in_odom[2, 0],
                          quat_baselink_in_odom[0, 0], quat_baselink_in_odom[1, 0], quat_baselink_in_odom[2, 0], quat_baselink_in_odom[3, 0]]))

            # (3) get odom in map
            # T_odom_in_map = T_baselink_in_map / T_baselink_in_odom (right divide)
            T_odom_in_map = np.linalg.lstsq(T_baselink_in_odom.T, T_baselink_in_map.T, rcond=None)[0].T
            trans_quat_odom_in_map = transform_matrix_to_trans_quat(T_odom_in_map)
            trans_odom_in_map = trans_quat_odom_in_map[0:3, 0]
            quat_odom_in_map = trans_quat_odom_in_map[3:, 0]

            self.tf_broadcast(trans_odom_in_map, quat_odom_in_map, 'odom', 'map')

    def fdi_gps_cb(self, msg):
        if self.gps_loc_on:
            # lla = np.array([np.deg2rad(msg.latitude), np.deg2rad(msg.longitude), msg.altitude])
            lla = np.array([msg.latitude, msg.longitude, msg.altitude])
            track = -np.deg2rad(msg.track) # the direction of gps angle is opposite of Z axis up, maybe it is NED, not ENU
            theta = track + np.pi / 2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            # (1) get base_link in map
            #  x, y, z coords for East, North, Up frame in map
            trans_gps_in_enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])
            # Quaternion for East, North, Up frame in map
            ypr_gps_in_enu = np.array([theta, 0, 0])

            T_gps_in_enu = transform_trans_ypr_to_matrix(np.array([trans_gps_in_enu[0], trans_gps_in_enu[1], 0.0,
                                                                   ypr_gps_in_enu[0], ypr_gps_in_enu[1],
                                                                   ypr_gps_in_enu[2]]))

            T_baselink_in_enu = T_gps_in_enu @ self.T_baselink_in_gps

            # T_baselink_in_map = T_map_in_enu\T_baselink_in_enu: base_link in map's coordinate (left divide)
            T_baselink_in_map = np.linalg.lstsq(self.T_map_in_enu, T_baselink_in_enu, rcond=None)[0]

            # (2) get base_link in odom
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return
            trans_baselink_in_odom = np.array([[trans[0]], [trans[1]], [trans[2]]])
            quat_baselink_in_odom = np.array([[rot[0]], [rot[1]], [rot[2]], [rot[3]]])  # rot (x, y, z, w)

            T_baselink_in_odom = transform_trans_quat_to_matrix(
                np.array([trans_baselink_in_odom[0, 0], trans_baselink_in_odom[1, 0], trans_baselink_in_odom[2, 0],
                          quat_baselink_in_odom[0, 0], quat_baselink_in_odom[1, 0], quat_baselink_in_odom[2, 0],
                          quat_baselink_in_odom[3, 0]]))

            # (3) get odom in map
            # T_odom_in_map = T_baselink_in_map / T_baselink_in_odom (right divide)
            T_odom_in_map = np.linalg.lstsq(T_baselink_in_odom.T, T_baselink_in_map.T, rcond=None)[0].T
            trans_quat_odom_in_map = transform_matrix_to_trans_quat(T_odom_in_map)
            trans_odom_in_map = trans_quat_odom_in_map[0:3, 0]
            quat_odom_in_map = trans_quat_odom_in_map[3:, 0]

            self.tf_broadcast(trans_odom_in_map, quat_odom_in_map, 'odom', 'map')


def main(args):
    rospy.init_node('gps_loc_node', anonymous=True)
    gl = gps_localizer()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
