#!/usr/bin/env python
"""
# target_asap.py

Set up Target parameters.
"""

import time
import rospy
import rospkg
import math
import tf.transformations as tft

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")+"/"


def target_execute_test(traj_file, bee_topic_prefix, test_number=-1, ground='false', sim=False):
	""" Run a target test.
	"""
	print("""==========================================================
	         [TARGET_ASAP]: Running...""")
	rospy.set_param('/td/ground', ground)  # options are: ['false', 'true']
	rospy.set_param('td/role', "target")

	# Set initial position
	if ground=='true':
		r_RI = [0.0, 0.0, 0.0]  # the test volume reference frame (TVR) wrt INERTIAL frame
		r_TR = [0.0, -0.5, -0.7]  # Target position [x y z] wrt TVR frame
		q_TR = [1.0, 0.0, 0.0, 0.0]  # quaternion [qx qy qz qw wrt TVR frame]

		set_params_IC(r_RI, r_TR, q_TR)
	else:
		r_RI_ISS_x = 10.9
		r_RI_ISS_y = -8.15
		r_RI_ISS_z = 4.9
		r_RI = [r_RI_ISS_x, r_RI_ISS_y, r_RI_ISS_z] # the test volume reference frame (TVR) wrt INERTIAL frame
		rospy.set_param('/td/r_RI_ISS_x', r_RI_ISS_x)
		rospy.set_param('/td/r_RI_ISS_y', r_RI_ISS_y)
		rospy.set_param('/td/r_RI_ISS_z', r_RI_ISS_z)

		if test_number >= 100:  # ISS case
			axis = ""
			if str(test_number)[0] == '1':  # x-axis
				axis = "x"
			elif str(test_number)[0] == '2':  # y-axis
				axis = "y"
			elif str(test_number)[0] == '3':  # z-axis
				axis = "z"

			if str(test_number)[1] == '1':
				tumble = "tri"
			elif str(test_number)[1] == '2':
				tumble = "flat_spin"
			elif str(test_number)[1] == '3':
				tumble = "none"
			else:
				tumble = "none"
			select_IC_by_axis_and_tumble(axis, tumble, r_RI)
		else:  # use y axis for the other cases
			r_TR = [0.0, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
			q_TR = [0.0, 0.0, 0, 1]  # quaternion [qx qy qz qw wrt TVR frame]
			set_params_IC(r_RI, r_TR, q_TR)

	rospy.set_param('/td/target/after_reg_pause', 2.0)  # seconds to wait after regulation is complete before starting test
	rospy.set_param('/td/target_coordinator/target_traj_file', traj_file)

	print_test_msg(test_number)  # optional


def set_params_IC(r_RI, r_TR, q_TR):
	rospy.set_param('/td/target/x_start', r_RI[0] + r_TR[0])
	rospy.set_param('/td/target/y_start', r_RI[1] + r_TR[1])
	rospy.set_param('/td/target/z_start', r_RI[2] + r_TR[2])
	rospy.set_param('/td/target/qx_start', q_TR[0]) # x
	rospy.set_param('/td/target/qy_start', q_TR[1]) # y
	rospy.set_param('/td/target/qz_start', q_TR[2]) # z
	rospy.set_param('/td/target/qw_start', q_TR[3]) # w

	eul_TR = tft.euler_from_quaternion(q_TR, 'rxyz')  # as a quaternion
	rospy.set_param('/td/target/eulx_start', eul_TR[0])  # [X Y Z] rotating frame
	rospy.set_param('/td/target/euly_start', eul_TR[1])
	rospy.set_param('/td/target/eulz_start', eul_TR[2])


def select_IC_by_axis_and_tumble(axis, tumble, r_RI):
	""" Select initial conditions.
	"""
	if axis == "x":
		r_TR = [0.15, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
		#q_TR = [0.0, 0.0, 1.0, 0.0]  # quaternion [qx qy qz qw wrt TVR frame]
		q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]
		set_params_IC(r_RI, r_TR, q_TR)
	elif axis == "y" and tumble == "flat_spin":
		r_TR = [0.0, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
		#q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_TR = [0.0, 0.0, -0.7071, 0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		set_params_IC(r_RI, r_TR, q_TR)
	elif axis == "y" and (tumble == "tri" or tumble == "none"):
		r_TR = [0.0, -1.5, 0.0]  # Target position [x y z] wrt TVR frame
		#q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]
		set_params_IC(r_RI, r_TR, q_TR)
	elif axis == "z" and tumble == "flat_spin":
		r_TR = [0.0, -1.5, 0.25]  # Target position [x y z] wrt TVR frame
		#q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_TR = [0.0, -0.7071, 0.0, 0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		set_params_IC(r_RI, r_TR, q_TR)
	elif axis == "z" and (tumble == "tri" or tumble == "none"):
		r_TR = [0.0, -1.5, 0.25]  # Target position [x y z] wrt TVR frame
		#q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
		q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]
		set_params_IC(r_RI, r_TR, q_TR)

	rospy.set_param('/td/target/tumble_type', tumble)


def print_test_msg(test_number):
	if test_number==-1:  # stop execution
		print('[TARGET_ASAP] Killing test...')
	else:  # software launch checkout
		print('[TARGET_ASAP] Running test...')
