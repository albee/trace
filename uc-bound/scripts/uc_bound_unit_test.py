#!/usr/bin/env python

import time
import rospy
import rospkg
import math

import argparse
from std_msgs.msg import String
from threading import Thread

# For rosbag and CTL-C
import subprocess, shlex
from signal import signal, SIGINT, pause
from sys import exit

DATA_PATH = "/home/charles/tumbledock_ws/freeflyer-shared-td/develop/uc_bound/data/"
TOPICS = "/honey/targ_est_att /honey/targ_est_w /honey/targ_est_covatt /honey/targ_est_covw /honey/uc_bound /honey/num_trials /honey/sigma_level"

"""
uc_bound test function:
"""
def uc_bound_unit_test():
	# For unit testing, use DLR LUT trajectory #4
	traj_file = "/home/charles/tumbledock_ws/freeflyer-shared-td/develop/data/input/DLR_chaser_LUT_4_20_20/DATA/"
	bee_topic_prefix = '/honey/'

	# Start the test
	print('Starting UC Bound Unit Test: ')
	rospy.set_param(bee_topic_prefix+'td/chaser_traj_file', traj_file)
	rospy.set_param(bee_topic_prefix+'td/instruct', 'move')

"""
Spool up rosbag recording
"""
def rosbag_start():
	# For local machine. Need to modify for hardware.
	command = "rosbag record -O "+DATA_PATH+"uc_bound_unit_test.bag "+TOPICS
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



if __name__ == "__main__":
	signal(SIGINT, handler)  # for ctl-c handling
	# rosbag_start()

	Thread(target = uc_bound_unit_test()).start()
	rospy.init_node('uc_bound_unit_test', anonymous=True)
	rospy.spin()
