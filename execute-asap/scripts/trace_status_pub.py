#!/usr/bin/env python3

import rospy
from trace_msgs.msg import TDStatus
import time

def pub_status():
    msg = TDStatus()
    msg.stamp = rospy.get_rostime()
    msg.test_number = 0
    msg.chaser_coord_ok = True
    msg.target_coord_ok = True
    msg.slam_activate = True
    msg.motion_planner_interface_activate = True
    msg.uc_bound_activate = True
    msg.chaser_regulate_finished = True
    msg.target_regulate_finished = True
    msg.motion_plan_finished = True
    msg.motion_plan_wait_time = 1.0
    msg.uc_bound_finished = True
    msg.mrpi_finished = True
    msg.traj_finished = True
    msg.test_finished = True
    msg.default_control = False
    msg.td_control_mode = "debug"
    msg.td_state_mode = "ekf"
    msg.td_flight_mode = "nominal"

    time.sleep(0.5)
    print('msg sent...')
    pub = rospy.Publisher('/td/status', TDStatus, queue_size=10, latch=True)
    pub.publish(msg)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pub_node')
    pub_status()
