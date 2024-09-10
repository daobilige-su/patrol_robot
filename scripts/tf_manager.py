#! /usr/bin/env python

import rospy
import sys
import tf


class TFManager:
    def __init__(self):
        self.send_rate = rospy.Rate(10)  # send with 10 hz
        self.br = tf.TransformBroadcaster()

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    def send_tf(self):
        self.tf_broadcast([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
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
