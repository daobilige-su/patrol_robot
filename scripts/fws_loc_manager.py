#! /usr/bin/env python

import rospy
import sys
import tf
import yaml
import numpy as np
from transform_tools import *
import rospkg
from PIL import Image
from std_srvs.srv import SetBool

class LocManager:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot') + '/'

        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        self.manager_rate = rospy.Rate(10)

        # servers
        rospy.wait_for_service('GpsLocOn')
        self.gps_loc_req = rospy.ServiceProxy('GpsLocOn', SetBool)
        rospy.logwarn('GpsLocOn service connected.')
        rospy.wait_for_service('AlsLocOn')
        self.als_loc_req = rospy.ServiceProxy('AlsLocOn', SetBool)
        rospy.logwarn('AlsLocOn service connected.')

        # set map params
        self.env_map_yaml_file = self.param['loc']['env_map']
        self.loc_src_map_yaml_file = self.param['loc']['loc_src_map']

        with open(self.pkg_path+'map/'+self.env_map_yaml_file, 'r') as file:
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

    def update_map_and_loc(self):
        res = self.gps_loc_req(True)
        rospy.loginfo('GPS Loc On request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)
        res = self.als_loc_req(False)
        rospy.loginfo('ALS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)

        rospy.sleep(5)

        res = self.gps_loc_req(False)
        rospy.loginfo('GPS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)
        res = self.als_loc_req(True)
        rospy.loginfo('ALS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)

        rospy.sleep(5)
        pass


def main(args):
    rospy.init_node('fws_loc_manager_node', anonymous=True)
    loc_manager = LocManager()
    try:
        while not rospy.is_shutdown():
            loc_manager.update_map_and_loc()
            loc_manager.manager_rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
