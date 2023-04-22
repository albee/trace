#!/usr/bin/env python

import time
import rospy
import rospkg
import math

DATA_PATH = "/home/charles/tumbledock_ws/freeflyer-shared-td/develop/target_coordinator/data/"
TOPICS = "/loc/truth/pose /loc/truth/twist /gnc/ctl/setpoint /DLR_traj_truth"

import argparse
from std_msgs.msg import String
from threading import Thread

# For rosbag and CTL-C
import subprocess, shlex
from signal import signal, SIGINT, pause
from sys import exit

"""
tumble_test function:
"""
def target_unit_test():
	
	# For unit testing, compare with Roberto's DLR trajectory (Motion_Target.dat)
	target_att0_1 = 0.0   # Euler 313 angles
	target_att0_2 = 0.0
	target_att0_3 = 0.0           
	target_w0_x = 0.0     # rad/s
	target_w0_y = 0.0617067
	target_w0_z = 0.0617067
	traj_file = "/home/charles/tumbledock_ws/freeflyer-shared-td/develop/data/input/sample_trajectories/Motion_Target.dat"
	bee_topic_prefix = '/'

	# 1. Start the tumbling controller node to tumble the Astrobee and record state/command data
	print('Starting Target Coordinator Unit Test: ')
	rospy.set_param(bee_topic_prefix+'td/target_traj_file', traj_file)
	rospy.set_param(bee_topic_prefix+'td/instruct', 'test0')
	rospy.set_param(bee_topic_prefix+'target_att0_1', target_att0_1)
	rospy.set_param(bee_topic_prefix+'target_att0_2', target_att0_2)
	rospy.set_param(bee_topic_prefix+'target_att0_3', target_att0_3)
	rospy.set_param(bee_topic_prefix+'target_w0_x', target_w0_x)
	rospy.set_param(bee_topic_prefix+'target_w0_y', target_w0_y)
	rospy.set_param(bee_topic_prefix+'target_w0_z', target_w0_z)


"""
Spool up rosbag recording
"""
def rosbag_start():
	# For local machine. Need to modify for hardware.
	command = "rosbag record -O "+DATA_PATH+"target_unit_test.bag "+TOPICS
	command = shlex.split(command)
	astro_bag = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)  # so SIGINT is caught by the parent
	print("Rosbag recording started")
	

"""
Kill rosbag on ctl-c
"""
def rosbag_stop():
	astro_bag.send_signal(subprocess.signal.SIGINT)

"""
Catch ctl-c
"""
def handler(signal_received, frame):
	print('\nSIGINT or CTRL-C detected, killing rosbags...')
	try:
		rosbag_stop()
		print('...complete.')
	except:
		print('...no rosbag started.')


#### Main script
if __name__ == "__main__":
	signal(SIGINT, handler)  # for ctl-c handling
	rosbag_start()

	Thread(target = target_unit_test()).start()

	rospy.init_node('target_unit_test', anonymous=True)
	rospy.spin()



	

