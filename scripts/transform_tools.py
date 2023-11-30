#!/usr/bin/env python
# -*- coding: utf-8 -*-

# sudo apt-get install python3-numpy python3-scipy python3-matplotlib python3-pandas python3-sympy python3-nose
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

def transform_matrix_from_trans_ypr(trans_ypr):
    trans_ypr = trans_ypr.reshape((-1,1))
    # R.from_euler('ZYX', [y,p,r],degrees=true), 'ZYX' means extrinsic, 'zyx' means intrinsic, we use extrinsic,
    # it basically means World to Object or Object to World
    r = R.from_euler('ZYX', [trans_ypr[3,0]*(180.0/math.pi),trans_ypr[4,0]*(180.0/math.pi),trans_ypr[5,0]*(180.0/math.pi)], degrees=True)
    #r_M = r.as_dcm()
    r_M = r.as_matrix()

    M = np.zeros((4,4))
    M[3, 3] = 1
    M[0:3,0:3] = r_M
    M[0:3,3:4] = np.array([[trans_ypr[0,0]],[trans_ypr[1,0]],[trans_ypr[2,0]]])

    return M


def quat2ypr(quat): # quat: (x, y, z, w)
    quat = quat.reshape((-1, 1))
    # in R.from_quat(quat), quat: (x, y, z, w)
    r = R.from_quat([quat[0,0], quat[1,0], quat[2,0], quat[3,0]])
    ypr = r.as_euler('ZYX') # list [y p r]

    ypr_np = np.array([[ypr[0]],[ypr[1]],[ypr[2]]])

    return ypr_np


def ypr2quat(ypr):
    ypr = ypr.reshape((-1, 1))
    r = R.from_euler('ZYX', [ypr[0,0], ypr[1,0], ypr[2,0]])
    # in R.as_quat(quat), quat: (x, y, z, w)
    quat = r.as_quat()

    quat_np = np.array([[quat[0]],[quat[1]],[quat[2]],[quat[3]]])

    return quat_np #(x,y,z,w)


def transform_matrix_to_pose_trans_ypr(M):
    r_M = M[0:3,0:3]
    trans = M[0:3,3:4]

    #r = R.from_dcm(r_M)
    r = R.from_matrix(r_M)
    ypr = r.as_euler('ZYX')

    trans_ypr = np.array([[trans[0,0]],[trans[1,0]],[trans[2,0]],[ypr[0]],[ypr[1]],[ypr[2]]])

    return trans_ypr


def wrap_to_pi(rad):
    rad = rad % (math.pi*2.0)
    if rad<=(-1.0)*math.pi:
        rad = rad + math.pi*2.0
    elif rad>math.pi:
        rad = rad - math.pi*2.0

    return rad