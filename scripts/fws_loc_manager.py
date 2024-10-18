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
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

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
        self.env_map_img = Image.open(env_map_pgm_filename)
        env_map_np = np.flipud(np.asarray(self.env_map_img).astype(float))
        env_map_np_p = (255.0 - env_map_np) / 255.0
        env_map_msg_data_np = -1 * np.ones(env_map_np.shape, dtype=np.int8)
        env_map_msg_data_np[env_map_np_p > self.env_map_yaml['occupied_thresh']] = 100
        env_map_msg_data_np[env_map_np_p < self.env_map_yaml['free_thresh']] = 0
        self.env_map_np = env_map_msg_data_np.copy()
        self.env_map_size = self.env_map_np.size # w,h
        self.env_map_res = self.env_map_yaml['resolution']
        self.env_map_ct_m = [-self.env_map_yaml['origin'][0], -self.env_map_yaml['origin'][1]] # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        self.env_map_ct_px = [self.env_map_ct_m[0]/self.env_map_res, self.env_map_ct_m[1]/self.env_map_res]

        # with open(self.env_map_yaml_file, 'r') as file:
        #     self.env_map_yaml = yaml.safe_load(file)
        # env_map_pgm_filename = self.pkg_path + 'map/' + self.env_map_yaml['image']
        # self.env_map_img = Image.open(env_map_pgm_filename)
        # self.env_map_size = self.env_map_img.size  # w,h
        # self.env_map_res = self.env_map_yaml['resolution']
        # self.env_map_ct_m = [-self.env_map_yaml['origin'][0],
        #                      self.env_map_size[1] * self.env_map_res + self.env_map_yaml['origin'][
        #                          1]]  # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        # self.env_map_ct_px = [self.env_map_ct_m[0] / self.env_map_res, self.env_map_ct_m[1] / self.env_map_res]

        self.env_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

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

        env_map_msg = OccupancyGrid()
        env_map_msg.header.stamp = rospy.get_rostime()
        env_map_msg.header.frame_id = 'map'

        env_map_msg.info.map_load_time = rospy.get_rostime()
        env_map_msg.info.resolution = self.env_map_res
        env_map_msg.info.width = self.env_map_size[0]
        env_map_msg.info.height = self.env_map_size[1]

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1.0
        env_map_msg.info.origin = pose

        env_map_np_p = (255.0-self.env_map_np)/255.0
        env_map_msg_data_np = -1*np.ones(self.env_map_np.shape, dtype=np.int8)
        env_map_msg_data_np[env_map_np_p > self.env_map_yaml['occupied_thresh']] = 100
        env_map_msg_data_np[env_map_np_p < self.env_map_yaml['free_thresh']] = 0
        env_map_msg.data = env_map_msg_data_np.reshape((-1,)).tolist()

        self.env_map_pub.publish(env_map_msg)

        return


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
