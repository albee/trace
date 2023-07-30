#!/usr/bin/env python
"""
# chaser_asap.py

Set up Chaser parameters.
"""

import time
import rospy
import rospkg
import math
import argparse
from std_msgs.msg import String
import tf.transformations as tft

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")+"/"

def chaser_execute_test(traj_file, bee_topic_prefix, test_number=-1, ground='false', sim=False):
	"""
	Run a chaser test.
	"""
	print('[CHASER_ASAP]: Running...')
	# Set baseline parameters
	rospy.set_param('/td/ground', ground)  # options are: ['false', 'true']
	rospy.set_param('/td/casadi_on_target', False)
	rospy.set_param('td/role', "chaser")

	rospy.set_param('/td/chaser/after_reg_pause', 2.0)  # seconds to wait after regulation is complete before starting test
	rospy.set_param('/td/chaser_traj_file', traj_file)  # tell Chaser where to look for traj file

	if ground=="true":
		r_RI = [0.0, 0.0, 0.0]  # the test volume reference frame (TVR) wrt INERTIAL frame

		r_TR = [0.0, -0.5, -0.7]  # Target position [x y z] wrt TVR frame
		r_CR = [0.0, 0.6, -0.7]  # Chaser position wrt TVR frame

		q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]
		q_CR = [0.0, 0.0, -0.7071068, 0.7071068]  # quaternion [qx qy qz qw wrt TVR frame]

		set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR)
	else:
		r_RI_ISS_x = 10.9
		r_RI_ISS_y = -8.15
		r_RI_ISS_z = 4.9
		r_RI = [r_RI_ISS_x, r_RI_ISS_y, r_RI_ISS_z]  # the test volume reference frame (TVR) wrt INERTIAL frame. UNCHANGED
		rospy.set_param('/td/r_RI_ISS_x', r_RI_ISS_x)
		rospy.set_param('/td/r_RI_ISS_y', r_RI_ISS_y)
		rospy.set_param('/td/r_RI_ISS_z', r_RI_ISS_z)

		if test_number >= 100:  # Standard tests
			axis = ""
			if str(test_number)[0] == '1':  # x-axis
				axis = "x"
			elif str(test_number)[0] == '2':  # y-axis
				axis = "y"
			elif str(test_number)[0] == '3':  # z-axis
				axis = "z"
			select_IC_by_axis(axis, r_RI)
		else:  # Unit tests
			r_TR = [0.0, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
			r_CR = [0.0, 0.0, 0.0]  # Chaser position wrt TVR frame

			q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
			q_CR = [0.0, 0.0, -0.7071, 0.7071]  # quaternion [qx qy qz qw wrt TVR frame]

			set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR)

	print_test_msg(test_number)  # optional


def set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR):
	# these params are ONLY used for Roberto's old format
	rospy.set_param('/td/x_TI', r_RI[0] + r_TR[0])  # x Target wrt Inertial, INERTIAL
	rospy.set_param('/td/y_TI', r_RI[1] + r_TR[1])  # y
	rospy.set_param('/td/z_TI', r_RI[2] + r_TR[2])  # z

	rospy.set_param('/td/x_CT', r_CR[0] - r_TR[0])  # Chaser wrt Target fixed frame
	rospy.set_param('/td/y_CT', r_CR[1] - r_TR[1])
	rospy.set_param('/td/z_CT', r_CR[2] - r_TR[2])

	# these params are what matter for actual testing, wrt INERTIAL frame
	rospy.set_param('/td/chaser/x_start', r_RI[0] + r_CR[0])  # x
	rospy.set_param('/td/chaser/y_start', r_RI[1] + r_CR[1])  # y
	rospy.set_param('/td/chaser/z_start', r_RI[2] + r_CR[2])  # z
	rospy.set_param('/td/chaser/qx_start', q_CR[0])  # x
	rospy.set_param('/td/chaser/qy_start', q_CR[1])  # y
	rospy.set_param('/td/chaser/qz_start', q_CR[2])  # z
	rospy.set_param('/td/chaser/qw_start', q_CR[3])  # w

	rospy.set_param('/td/chaser/targ_offset_x', r_RI[0] + r_TR[0])  # x
	rospy.set_param('/td/chaser/targ_offset_y', r_RI[1] + r_TR[1])  # y
	rospy.set_param('/td/chaser/targ_offset_z', r_RI[2] + r_TR[2])  # z

	eul_CR = tft.euler_from_quaternion(q_CR, 'rxyz')  # as a quaternion
	rospy.set_param('/td/chaser/eulx_start', eul_CR[0])  # [X Y Z] rotating frame
	rospy.set_param('/td/chaser/euly_start', eul_CR[1])
	rospy.set_param('/td/chaser/eulz_start', eul_CR[2])


def select_IC_by_axis(axis, r_RI):
	""" Select initial conditions.
	"""
	if axis == "x":
		print(r_RI)
		r_TR = [0.15, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
		r_CR = [-0.55, -1.5, 0.0]  # Chaser position wrt TVR frame

		q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]
		q_CR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]

		set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR)
	elif axis == "y":
		r_TR = [0.0, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
		r_CR = [0.0, 0.0, 0.0]  # Chaser position wrt TVR frame

		q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_CR = [0.0, 0.0, -0.7071, 0.7071]  # quaternion [qx qy qz qw wrt TVR frame]

		set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR)
	elif axis == "z":
		r_TR = [0.0, -1.5, 0.25]  # Target position [x y z] wrt TVR frame
		r_CR = [0, -1.5, -0.5]  # Chaser position wrt TVR frame

		q_TR = [0.0, 0.7071, 0.0, 0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_CR = [0.0, 0.7071, 0.0, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]

		set_params_IC(r_RI, r_TR, r_CR, q_TR, q_CR)

def print_test_msg(test_number):
	if test_number==-1:  # stop execution
		print('[CHASER_ASAP] Killing test...')
	else:  # software launch test
		print('[CHASER_ASAP] Running test...')

if __name__ == "__main__":
	# Parse input
	DATA_PATH = rospack.get_path("data")+"/"
	TRAJ_FILE = "input/sample-trajectories/TEST12-ISS/"
	bee_topic_prefix = "/"

	parser = argparse.ArgumentParser(description='Run a desired test session.',
		usage='%(prog)s [options]')
	parser.add_argument('-g', '--ground', action='store_true', help='-g to indicate ground. Defaults to ISS.')
	parser.add_argument('-s', '--sim', action='store_true', help='-s to indicate simulation. Defaults to hardware.')
	parser.add_argument('test_num', metavar='TEST', type=String, nargs=1, help='A test number to run.')
	args = parser.parse_args()

	if args.sim == True:
		SIM = "sim"
		bee_topic_prefix = "/queen/"
	else:
		SIM = "hardware"
	if args.ground == True:
		GROUND = "true"
	else:
		GROUND = "false"
	TEST_NUMBER = int(args.test_num[0].data)

	# Start ROS, send command
	rospy.init_node('execute_asap', anonymous=True)
	chaser_execute_test(DATA_PATH + TRAJ_FILE, bee_topic_prefix, TEST_NUMBER, GROUND)
