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
import copy
import cv2


class LocMapCreator:
    def __init__(self):
        self.dist_from_explored_map = 5

        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot') + '/'

        env_map_yaml_file = rospy.get_param('/env_map_yaml_file')

        # set map params
        self.env_map_yaml_file = env_map_yaml_file

        self.env_map = self.load_map(self.env_map_yaml_file)
        self.loc_src_map = MapClass()
        self.dist_in_pix = int(self.dist_from_explored_map/self.env_map.res)

        # map pub
        self.env_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.loc_src_map_pub = rospy.Publisher('/loc_src_map', OccupancyGrid, queue_size=1)

    def create_map(self):
        # for debugging
        # import matplotlib.pyplot as plt
        # import matplotlib as mpl
        # mpl.use('TkAgg')
        # plt.imshow(image_seg_bn, cmap='gray')
        # plt.show()

        self.loc_src_map = copy.deepcopy(self.env_map)
        self.loc_src_map.yaml['image'] = self.loc_src_map.yaml['image'][:-4] + '_generated_loc_src_map.pgm'

        # img = self.loc_src_map.img
        env_explored_map_np = self.loc_src_map.np.copy().astype(float)  # 100: obstacle, 0: free, -1: unkown
        env_explored_map_np[env_explored_map_np>-0.5] = 1
        env_explored_map_np[env_explored_map_np<-0.5] = 0
        loc_src_map_np = 100*np.ones(env_explored_map_np.shape).astype(np.int8)  # 0: lidar, 100: gps

        # the following code takes too long time
        # for u in range(self.loc_src_map.size[0]):
        #     for v in range(self.loc_src_map.size[1]):
        #         u_min = max(u-self.dist_in_pix, 0)
        #         u_max = min(u+self.dist_in_pix, self.loc_src_map.size[0])
        #         v_min = max(v - self.dist_in_pix, 0)
        #         v_max = min(v + self.dist_in_pix, self.loc_src_map.size[1])
        #
        #         local_env_explored_map = env_explored_map_np[v_min:v_max, u_min:u_max]
        #         if np.sum(local_env_explored_map)>0.5:
        #             loc_src_map_np[v, u] = 100

        # use uniform kernel and 2D conv (kernel size of self.dist_from_explored_map) for blurring, and set
        # element > min blur (1/(self.dist_in_pix*self.dist_in_pix)) to be 0
        env_explored_map_np_conv = cv2.filter2D(env_explored_map_np, -1, (1/(self.dist_in_pix*self.dist_in_pix))*np.ones((self.dist_in_pix, self.dist_in_pix)))
        loc_src_map_np[env_explored_map_np_conv>(1/(self.dist_in_pix*self.dist_in_pix))] = 0

        self.loc_src_map.np = loc_src_map_np.copy()

        # save map to pgm file
        img_100_np = np.flipud(self.loc_src_map.np)
        img_255_np = np.zeros(img_100_np.shape).astype(np.uint8)
        img_255_np[img_100_np==0] = 255  # free space to 255 in image
        img = Image.fromarray(img_255_np)
        self.loc_src_map.img = img.copy()

        # write img and yaml file
        self.loc_src_map.img.save(self.pkg_path + 'map/' + self.loc_src_map.yaml['image'])
        yaml_file_name = self.loc_src_map.yaml['image'][0:-4] + '.yaml'
        with open(self.pkg_path + 'map/' + yaml_file_name, 'w') as file:
            yaml.dump(self.loc_src_map.yaml, file, default_flow_style=False, sort_keys=False)

        rospy.logwarn('loc_src_map: ' + self.pkg_path + 'map/' + yaml_file_name + ' is generated.')

        # publish msg
        rospy.logwarn('waiting 5s to send ros map msgs... ')
        rospy.sleep(5)
        loc_src_map_msg = self.create_map_msg(self.loc_src_map.np, self.loc_src_map.res, [0, 0, 0])
        self.loc_src_map_pub.publish(loc_src_map_msg)
        env_map_msg = self.create_map_msg(self.env_map.np, self.env_map.res, [0, 0, 0])
        self.env_map_pub.publish(env_map_msg)
        rospy.logwarn('map msgs sent.')

        return

    @staticmethod
    def create_map_msg(map_np, map_res, pose_2d):
        # test env_map publishing
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.get_rostime()
        map_msg.header.frame_id = 'map'

        map_msg.info.map_load_time = rospy.get_rostime()
        map_msg.info.resolution = map_res
        map_msg.info.width = map_np.shape[1]
        map_msg.info.height = map_np.shape[0]

        pose = Pose()
        pose.position.x = pose_2d[0]
        pose.position.y = pose_2d[1]
        ypr = np.array([pose_2d[2], 0, 0])
        quat = ypr2quat(ypr)
        pose.orientation.z = quat[2, 0]
        pose.orientation.w = quat[3, 0]
        map_msg.info.origin = pose

        map_msg.data = map_np.reshape((-1,)).tolist()

        return map_msg

    def load_map(self, yaml_file):
        mapCl = MapClass()
        with open(self.pkg_path + 'map/' + yaml_file, 'r') as file:
            mapCl.yaml = yaml.safe_load(file)
        pgm_filename = self.pkg_path + 'map/' + mapCl.yaml['image']
        mapCl.img = Image.open(pgm_filename)
        map_prob_np = np.flipud(np.asarray(mapCl.img).astype(float))
        if mapCl.yaml['negate'] == 0:
            map_prob_np_p = (255.0 - map_prob_np) / 255.0
        else:
            rospy.logerr('fws_loc_manager.py: negate=1 not implemented.')
            return None
        map_np = -1 * np.ones(map_prob_np.shape, dtype=np.int8)
        map_np[map_prob_np_p > mapCl.yaml['occupied_thresh']] = 100
        map_np[map_prob_np_p < mapCl.yaml['free_thresh']] = 0
        mapCl.np = map_np.copy()
        mapCl.size = np.array([mapCl.np.shape[1], mapCl.np.shape[0]])  # w,h
        mapCl.res = mapCl.yaml['resolution']
        # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        mapCl.ct_m = [-mapCl.yaml['origin'][0], -mapCl.yaml['origin'][1]]
        mapCl.ct_px = [mapCl.ct_m[0] / mapCl.res, mapCl.ct_m[1] / mapCl.res]

        rospy.logwarn('env_map: ' + self.pkg_path + 'map/' + yaml_file + ' has been loaded.')

        return mapCl


class MapClass:
    def __init__(self):
        self.yaml = None  # yaml file name
        self.img = None  # map as image (top left corner is the origin of coords, with x to left and y down, so it is upside down)
        self.np = None  # map as np (origin is bot left corner, x to left and y up, con)
        self.size = None  # [w, h]
        self.res = None  # [pix size in m]
        self.ct_m = None  # origin's coord in the frame of bot left corner, unit in m
        self.ct_px = None  # origin's coord in the frame of bot left corner, unit in pixel


def main(args):
    rospy.init_node('create_loc_map_from_env_map_node', anonymous=True)
    loc_map_creator = LocMapCreator()
    try:
        loc_map_creator.create_map()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
