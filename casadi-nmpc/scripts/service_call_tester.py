#!/usr/bin/env python3

import rospy
import numpy as np
from trace_msgs.srv import TDMRPI_srv
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

def test():
    print('Calling mrpi...')
    rospy.wait_for_service('mrpi')
    print('Service ready...')

    wx = 0.02
    wy = 0.02
    wz = 0.02
    wdx = 0.02
    wdy = 0.02
    wdz = 0.02

    u_max_x = 10.0
    u_max_y = 10.0
    u_max_z = 10.0

    w = Float64MultiArray()
    w.data = [wx, wy, wz, wdx, wdy, wdz]
    w.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    w.layout.dim[0].size = 6
    w.layout.dim[1].size = 1

    u_max = Float64MultiArray()
    u_max.data = [u_max_x, u_max_y, u_max_z]
    u_max.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    u_max.layout.dim[0].size = 3
    u_max.layout.dim[1].size = 1

    dt = 0.2
    mass = 10.0
    Q_pos_anc = 5
    Q_vel_anc = 50
    R_anc = 0.1

    try:
        z_poly_calc_srv = rospy.ServiceProxy('mrpi', TDMRPI_srv)
        res = z_poly_calc_srv(w, u_max, dt, mass, Q_pos_anc, Q_vel_anc, R_anc)  # uses srv inputs one-by-one

        K = np.asarray(res.K.data).reshape(res.K.layout.dim[0].size,res.K.layout.dim[1].size)
        Au = np.asarray(res.Au.data).reshape(res.Au.layout.dim[0].size,res.Au.layout.dim[1].size)
        bu = np.asarray(res.bu.data).reshape(res.bu.layout.dim[0].size,res.bu.layout.dim[1].size)
        AZ = np.asarray(res.AZ.data).reshape(res.AZ.layout.dim[0].size,res.AZ.layout.dim[1].size)
        bZ = np.asarray(res.bZ.data).reshape(res.bZ.layout.dim[0].size,res.bZ.layout.dim[1].size)

        print(K)
        print(Au)
        print(bu)
        print(AZ)
        print(bZ)

        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    test()
