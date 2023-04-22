#!/usr/bin/env python

from threading import Thread
import sys
import argparse
import rospy
from std_msgs.msg import String

import subprocess, shlex
from signal import signal, SIGINT, pause
from sys import exit
import time

if __name__ == "__main__":
	""" Publish on the topics GDS will use to activate nodes.
	"""
	parser = argparse.ArgumentParser(description='Run a desired test session.',
		usage='%(prog)s [options]')
	parser.add_argument('-g', '--ground', action='store_true', help='-g to indicate ground. Defaults to ISS.')
	parser.add_argument('-s', '--sim', action='store_true', help='-s to indicate simulation. Defaults to hardware.')
	parser.add_argument('test_num', metavar='TEST', type=String, nargs=1, help='A test number to run.')
	args = parser.parse_args()

	if args.sim == True:
		SIM = "sim"
	else:
		SIM = "hardware"
	if args.ground == True:
		GROUND = "true"
	else:
		GROUND = "false"
	TEST_NUMBER = int(args.test_num[0].data)

	print('GDS messages publishing. Press Ctrl+C to stop publishing.')
	#while not rospy.is_shutdown():
	rospy.set_param("/td/gds_sim", SIM)
	rospy.set_param("/td/gds_ground", GROUND)
	time.sleep(1)
	rospy.set_param("/td/gds_test_num", TEST_NUMBER)

	print('GDS parameters set.')
