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
from geometry_msgs.msg import PoseWithCovarianceStamped


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

        self.env_map = self.load_map(self.env_map_yaml_file)
        self.loc_src_map = self.load_map(self.loc_src_map_yaml_file)

        self.loc_state = 0  # 0: lidar, 1: gps
        self.enter_gps_region_dist = self.param['loc']['enter_gps_region_dist']

        # map pub
        self.env_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        # tf listener
        self.tf_listener = tf.TransformListener()

        # state change flag
        self.loc_state_change = 0

        # initialpose msg to restart als-ros
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    def update_map_and_loc(self):
        # # test loc sensor switching
        # self.set_gps_on()
        # rospy.sleep(5)
        # self.set_als_on()
        # rospy.sleep(5)
        #
        # # test env_map publishing
        # env_map_msg = self.create_map_msg(self.env_map.np, self.env_map.res, [0, 0, 0])
        # self.env_map_pub.publish(env_map_msg)

        # based on localization source map, determine which loc method (lidar or gps) will be used.
        loc_src_map = copy.copy(self.loc_src_map)
        env_map = copy.copy(self.env_map)
        enter_gps_region_dist_pix = self.enter_gps_region_dist / loc_src_map.res

        # listen to the latest tf for base_link in map
        try:
            # trans: [x, y, z], rot: [qx, qy, qz, qw]
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        ypr = quat2ypr(np.array(rot)).reshape((-1,))
        rob_pose = np.array([trans[0], trans[1], ypr[0]])

        rob_loc_px = [(rob_pose[0]/loc_src_map.res)+loc_src_map.ct_px[0], (rob_pose[1]/loc_src_map.res)+loc_src_map.ct_px[1]]

        u_min = max(int(rob_loc_px[0]-enter_gps_region_dist_pix), 0)
        u_max = min(int(rob_loc_px[0]+enter_gps_region_dist_pix), self.loc_src_map.size[0])
        v_min = max(int(rob_loc_px[1]-enter_gps_region_dist_pix), 0)
        v_max = min(int(rob_loc_px[1]+enter_gps_region_dist_pix), self.loc_src_map.size[1])

        local_loc_src_map = 100 - loc_src_map.np[v_min:v_max, u_min:u_max]

        # determine the localization source
        self.loc_state_change = 0
        if self.loc_state == 0:  # if previously using lidar loc
            # if robot is outside of the explored area + dist margin of the loc_src_map, use gps loc
            if np.sum(local_loc_src_map) > 1:
                pass
            else:
                self.loc_state = 1
                self.loc_state_change = 1
        elif self.loc_state == 1:  # if previously using gps loc
            # if robot enters the explored area of the loc_src_map, use lidar loc
            rob_loc_px_int = [int(rob_loc_px[0]), int(rob_loc_px[1])]
            if 0<=rob_loc_px_int[0]<=self.loc_src_map.size[0] and 0<=rob_loc_px_int[0]<=self.loc_src_map.size[1]:
                if 100-self.loc_src_map.np[rob_loc_px_int[0], rob_loc_px_int[1]]>1:
                    self.loc_state = 0
                    self.loc_state_change = 1
        else:
            rospy.logerr('unknown loc_state, returning ...')
            return

        # enable selected localization source
        if self.loc_state_change:
            if self.loc_state == 0:
                # set the current robot pose as initial pose of the als
                self.set_als_on()
                rospy.sleep(0.1)
                self.send_intialpose_to_als(rob_pose)
                rospy.logwarn('/initialpose msg of [%f, %f, %f] sent to reset als.' % (rob_pose[0], rob_pose[1], rob_pose[2]))
            elif self.loc_state == 1:
                self.set_gps_on()
            else:
                rospy.logerr('unknown loc_state, returning ...')
                return

        return

    def send_intialpose_to_als(self, rob_pose):
        pose_cov = PoseWithCovarianceStamped()

        pose_cov.header.stamp = rospy.get_rostime()
        pose_cov.header.frame_id = 'map'

        pose_cov.pose.pose.position.x = rob_pose[0]
        pose_cov.pose.pose.position.y = rob_pose[1]
        quat = ypr2quat([rob_pose[2], 0, 0]).reshape((-1,))
        pose_cov.pose.pose.orientation.z = quat[2]
        pose_cov.pose.pose.orientation.w = quat[3]

        xy_cov = 0.1**2
        yaw_cov = np.deg2rad(10)**2
        # cov is x, y, z, x_axis, y_axis, z_axis
        pose_cov.pose.covariance = [xy_cov, 0, 0, 0, 0, 0,
                                    0, xy_cov, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, yaw_cov]

        self.initialpose_pub.publish(pose_cov)

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

        return mapCl

    # selecting als for lidar localization
    def set_als_on(self):
        # set gps off
        res = self.gps_loc_req(False)
        rospy.loginfo('GPS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)
        # set als on
        res = self.als_loc_req(True)
        rospy.loginfo('ALS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)

        return

    # selecting gps localization
    def set_gps_on(self):
        # set gps on
        res = self.gps_loc_req(True)
        rospy.loginfo('GPS Loc On request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)
        # set als off
        res = self.als_loc_req(False)
        rospy.loginfo('ALS Loc Off request sent.')
        rospy.loginfo('Response is:')
        rospy.loginfo(res)

        return


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
