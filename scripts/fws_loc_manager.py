#! /usr/bin/env python

import rospy
import sys
import tf
import yaml
import numpy as np
from transform_tools import *
import rospkg
from PIL import Image


class LocManager:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot') + '/'

        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        self.manager_rate = rospy.Rate(10)

        # set map params
        self.env_map_yaml_file = self.param['loc']['env_map']
        self.loc_src_map_yaml_file = self.param['loc']['loc_src_map']

        with open(self.env_map_yaml_file, 'r') as file:
            self.env_map_yaml = yaml.safe_load(file)
        env_map_pgm_filename = self.pkg_path+'map/'+self.env_map_yaml['image']
        self.env_map_pgm = Image.open(env_map_pgm_filename)
        self.env_map_size = self.env_map_pgm.size # w,h
        self.env_map_res = self.env_map_yaml['resolution']
        self.env_map_ct_m = [-self.env_map_yaml['origin'][0], self.env_map_size[1]*self.env_map_res+self.env_map_yaml['origin'][1]] # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        self.env_map_ct_px = [self.env_map_ct_m[0]/self.env_map_res, self.env_map_ct_m[1]/self.env_map_res]

        # with open(self.env_map_yaml_file, 'r') as file:
        #     self.env_map_yaml = yaml.safe_load(file)
        # env_map_pgm_filename = self.pkg_path + 'map/' + self.env_map_yaml['image']
        # self.env_map_pgm = Image.open(env_map_pgm_filename)
        # self.env_map_size = self.env_map_pgm.size  # w,h
        # self.env_map_res = self.env_map_yaml['resolution']
        # self.env_map_ct_m = [-self.env_map_yaml['origin'][0],
        #                      self.env_map_size[1] * self.env_map_res + self.env_map_yaml['origin'][
        #                          1]]  # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        # self.env_map_ct_px = [self.env_map_ct_m[0] / self.env_map_res, self.env_map_ct_m[1] / self.env_map_res]





        trans_ypr_laserlink_in_baselink = np.array(self.param['tf']['tran_ypr_laserlink_in_baselink'])
        quat_laserlink_in_baselink = ypr2quat(trans_ypr_laserlink_in_baselink[3:])
        self.trans_quat_laserlink_in_baselink = np.array(
            [trans_ypr_laserlink_in_baselink[0], trans_ypr_laserlink_in_baselink[1], trans_ypr_laserlink_in_baselink[2],
             quat_laserlink_in_baselink[0, 0], quat_laserlink_in_baselink[1, 0], quat_laserlink_in_baselink[2, 0], quat_laserlink_in_baselink[3, 0]])




def main(args):
    rospy.init_node('fws_loc_manager_node', anonymous=True)
    loc_manager = LocManager()
    try:
        while not rospy.is_shutdown():
            # loc_manager.send_tf()
            loc_manager.manager_rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
