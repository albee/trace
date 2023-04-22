#!/usr/bin/env python

"""
This is the main entrypoint to running test scripts. Fill out the ASAP class and begin calling
scripts using execute_test().

Run using: pub_gds_topics.py.
"""

# ROS
from ff_msgs.msg import SignalState
import argparse
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trace_msgs.msg import TDStatus
from trace_msgs.msg import TDTestNumber
from threading import Thread
import time
import os

# For rosbag and CTL-C
import subprocess, shlex
from signal import signal, SIGINT, pause
from sys import exit
import yaml
from rosbag.bag import Bag

# For data dir determination. No more filling in paths!
import rospy
import rospkg
import sys

# for launching and killing nodelets
import roslaunch
import datetime

# Load in individual Astrobee ASAPs.
import chaser_asap
import target_asap

# Globals for sim and hardware.
rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")
TRAJ_GEN_PATH = rospack.get_path("motion_planner_interface")
BAG_PATH_SIM = DATA_PATH + "/output/rosbags/"  # must end in /, dir must exist!
BAG_PATH_HARDWARE = "/data/bags/"  # must end in /, dir must exist!
# Example: /data/bags/2021-04-14/bsharp/delayed/20210414_1701_phase1Loc_test_bag_0.bag

TD_TARGET_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/td_target.launch"
TD_CHASER_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/td_chaser.launch"

# Topics to record---no recording if empty. MUST have a space between topics!
TOPICS_SIM_CHASER_FULL = "/queen/gnc/ctl/command /queen/gnc/ekf /queen/gnc/ctl/setpoint /queen/hw/pmc/command /queen/loc/truth/pose /queen/loc/truth/twist /queen/hw/imu /queen/hw/depth_haz/points \
/td/mit_slam/timing_info /td/mit_slam/target_centroid /td/mit_slam/delta_pose /td/mit_slam/loop_delta_pose \
/td/mit_slam/chaser_pose /td/mit_slam/chaser_twist /td/mit_slam/target_pose /td/mit_slam/target_twist \
/td/mit_slam/pose_GT /td/mit_slam/pose_GC /td/mit_slam/t_WT /td/mit_slam/inertia \
/td/motion_planner_interface/chaser_traj_pose0 /td/motion_planner_interface/chaser_traj_twist0 /td/motion_planner_interface/target_traj_pose0 /td/motion_planner_interface/target_traj_twist0 \
/td/uc_bound/uc_bound /td/motion_planner_interface/mpdebug_numbers /td/motion_planner_interface/mpdebug_path /td/motion_planner_interface/mpdebug_violations \
/queen/td/tube_mpc/traj /queen/td/tube_mpc/debug /td/tube_mpc/mrpi /queen/loc/ar/features /queen/loc/ml/features /queen/loc/of/features /queen/graph_loc/state /queen/sparse_mapping/pose \
/queen/mob/flight_mode /queen/mgt/img_sampler/nav_cam/image_record /td/tube_mpc/traj_body /queen/td/tube_mpc/traj_updated /queen/td/status \
/td/motion_planner_interface/status /queen/inv_fam/appliedFandT /queen/ekf_observer"

TOPICS_SIM_CHASER_NOHAZ = "/queen/gnc/ctl/command /queen/gnc/ekf /queen/gnc/ctl/setpoint /queen/hw/pmc/command /queen/loc/truth/pose /queen/loc/truth/twist /queen/hw/imu \
/td/mit_slam/timing_info /td/mit_slam/target_centroid /td/mit_slam/delta_pose /td/mit_slam/loop_delta_pose \
/td/mit_slam/chaser_pose /td/mit_slam/chaser_twist /td/mit_slam/target_pose /td/mit_slam/target_twist \
/td/mit_slam/pose_GT /td/mit_slam/pose_GC /td/mit_slam/t_WT /td/mit_slam/inertia \
/td/motion_planner_interface/chaser_traj_pose0 /td/motion_planner_interface/chaser_traj_twist0 /td/motion_planner_interface/target_traj_pose0 /td/motion_planner_interface/target_traj_twist0 \
/td/uc_bound/uc_bound /td/motion_planner_interface/mpdebug_numbers /td/motion_planner_interface/mpdebug_path /td/motion_planner_interface/mpdebug_violations \
/queen/td/tube_mpc/traj /queen/td/tube_mpc/debug /td/tube_mpc/mrpi /queen/loc/ar/features /queen/loc/ml/features /queen/loc/of/features /queen/graph_loc/state /queen/sparse_mapping/pose \
/queen/mob/flight_mode /td/tube_mpc/traj_body /queen/td/tube_mpc/traj_updated /queen/td/status /td/motion_planner_interface/status /queen/inv_fam/appliedFandT /queen/ekf_observer"

TOPICS_SIM_TARG = "/bumble/gnc/ctl/command /bumble/gnc/ekf /bumble/gnc/ctl/setpoint /bumble/hw/pmc/command /bumble/loc/truth/pose /bumble/loc/truth/twist /bumble/td/tube_mpc/traj /bumble/td/tube_mpc/debug \
/bumble/loc/ar/features /bumble/loc/ml/features /bumble/loc/of/features /bumble/graph_loc/state sparse_mapping/pose \
/bumble/mob/flight_mode /bumble/td/status /bumble/hw/imu /bumble/inv_fam/appliedFandT /bumble/ekf_observer"

TOPICS_SIM_TARG_NAVCAM = "/bumble/gnc/ctl/command /bumble/gnc/ekf /bumble/gnc/ctl/setpoint /bumble/hw/pmc/command /bumble/loc/truth/pose /bumble/loc/truth/twist /bumble/td/tube_mpc/traj /bumble/td/tube_mpc/debug \
/bumble/loc/ar/features /bumble/loc/ml/features /bumble/loc/of/features /bumble/graph_loc/state sparse_mapping/pose \
/bumble/mob/flight_mode /bumble/mgt/img_sampler/nav_cam/image_record /bumble/td/status /bumble/hw/imu /bumble/inv_fam/appliedFandT /bumble/ekf_observer"

TOPICS_HARDWARE_CHASER_FULL = "/gnc/ctl/command /gnc/ekf /gnc/ctl/setpoint /hw/pmc/command /hw/imu /hw/depth_haz/points \
/td/mit_slam/timing_info /td/mit_slam/target_centroid /td/mit_slam/delta_pose /td/mit_slam/loop_delta_pose \
/td/mit_slam/chaser_pose /td/mit_slam/chaser_twist /td/mit_slam/target_pose /td/mit_slam/target_twist \
/td/mit_slam/pose_GT /td/mit_slam/pose_GC /td/mit_slam/t_WT /td/mit_slam/inertia \
/td/motion_planner_interface/chaser_traj_pose0 /td/motion_planner_interface/chaser_traj_twist0 /td/motion_planner_interface/target_traj_pose0 /td/motion_planner_interface/target_traj_twist0 \
/td/uc_bound/uc_bound /td/motion_planner_interface/mpdebug_numbers /td/motion_planner_interface/mpdebug_path /td/motion_planner_interface/mpdebug_violations \
/td/tube_mpc/traj /td/tube_mpc/debug /td/tube_mpc/mrpi /loc/ar/features /loc/ml/features /loc/of/features /graph_loc/state /sparse_mapping/pose \
/mob/flight_mode /mgt/img_sampler/nav_cam/image_record /td/status /td/tube_mpc/traj_body /td/tube_mpc/traj_updated /td/motion_planner_interface/status /inv_fam/appliedFandT /ekf_observer"

TOPICS_HARDWARE_CHASER_NOHAZ = "/gnc/ctl/command /gnc/ekf /gnc/ctl/setpoint /hw/pmc/command /hw/imu \
/td/mit_slam/timing_info /td/mit_slam/target_centroid /td/mit_slam/delta_pose /td/mit_slam/loop_delta_pose \
/td/mit_slam/chaser_pose /td/mit_slam/chaser_twist /td/mit_slam/target_pose /td/mit_slam/target_twist \
/td/mit_slam/pose_GT /td/mit_slam/pose_GC /td/mit_slam/t_WT /td/mit_slam/inertia \
/td/motion_planner_interface/chaser_traj_pose0 /td/motion_planner_interface/chaser_traj_twist0 /td/motion_planner_interface/target_traj_pose0 /td/motion_planner_interface/target_traj_twist0 \
/td/uc_bound/uc_bound /td/motion_planner_interface/mpdebug_numbers /td/motion_planner_interface/mpdebug_path /td/motion_planner_interface/mpdebug_violations \
/td/tube_mpc/traj /td/tube_mpc/debug /td/tube_mpc/mrpi /loc/ar/features /loc/ml/features /loc/of/features /graph_loc/state /sparse_mapping/pose \
/mob/flight_mode /td/status /td/tube_mpc/traj_body /td/tube_mpc/traj_updated /td/motion_planner_interface/status /inv_fam/appliedFandT /ekf_observer"

TOPICS_HARDWARE_TARG = "/gnc/ctl/command /gnc/ekf /gnc/ctl/setpoint /hw/pmc/command /td/tube_mpc/traj /td/tube_mpc/debug /loc/ar/features /loc/ml/features /loc/of/features /graph_loc/state /sparse_mapping/pose \
/mob/flight_mode /td/status /hw/imu /inv_fam/appliedFandT /ekf_observer"

TOPICS_HARDWARE_TARG_NAVCAM = "/gnc/ctl/command /gnc/ekf /gnc/ctl/setpoint /hw/pmc/command /td/tube_mpc/traj /td/tube_mpc/debug /loc/ar/features /loc/ml/features /loc/of/features /graph_loc/state /sparse_mapping/pose \
/mob/flight_mode /mob/flight_mode /mgt/img_sampler/nav_cam/image_record /td/status /hw/imu /inv_fam/appliedFandT /ekf_observer"

# rosbag name
ROSBAG_NAME = "td_standard_rosbag"  # this gets overwritten!

# Chaser trajectory files (or path to executable), indexed by test#
CHASER_TRAJS_GND_SIM = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"", # test2 target coordinator unit test
    3: DATA_PATH+"/input/TEST3-GND/", # test3 tube-MPC unit test (and gain switch)
    4: DATA_PATH+"/input/TEST4-GND/", # Test4 PD attitude control unit test
    5: TRAJ_GEN_PATH+"/bin/ground-traj3-public/", # test5 motion planner unit test
    6: DATA_PATH+"", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/chaser_slam_traj_ground.csv", # test7 SLAM unit test (chaser motion)
    8: TRAJ_GEN_PATH+"/bin/ground-traj3-public/", # test8 full pipeline: nominal MPC, EKF state mode
    9: TRAJ_GEN_PATH+"/bin/ground-traj3-public/", # test9 full pipeline: tube MPC, EKF state mode
    10: TRAJ_GEN_PATH+"/bin/ground-traj3-public/", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: TRAJ_GEN_PATH+"/bin/ground-traj3-public/", # test11 full pipeline: tube MPC, SLAM state mode
    12: DATA_PATH+"/input/TEST12-GND/",  # test12 predefined trajectory tube MPC with Astrobee noise levels
    13: TRAJ_GEN_PATH+"/bin/ground-traj3-public/",
    14: TRAJ_GEN_PATH+"/bin/ground-traj3-public/",
    15: DATA_PATH+"/input/TEST3-GND/",
    16: DATA_PATH+"/input/TEST3-GND/",
    77: DATA_PATH+"/input/TEST12-GND/"  # debug
    }

CHASER_TRAJS_GND_HARDWARE = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"", # test2 target coordinator unit test
    3: DATA_PATH+"/input/TEST3-GND/", # test3 tube-MPC unit test (and gain switch)
    4: DATA_PATH+"/input/TEST4-GND/", # Test4 PD attitude control unit test
    5: "/opt/roam/mp/", # test5 motion planner unit test
    6: DATA_PATH+"", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/chaser_slam_traj_ground.csv", # test7 SLAM unit test (chaser motion)
    8: "/opt/roam/mp/", # test8 full pipeline: nominal MPC, EKF state mode
    9: "/opt/roam/mp/", # test9 full pipeline: tube MPC, EKF state mode
    10:"/opt/roam/mp/", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: "/opt/roam/mp/", # test11 full pipeline: tube MPC, SLAM state mode
    12: DATA_PATH+"/input/TEST12-GND/", # test12 predefined trajectory tube MPC with Astrobee noise levels
    13: "/opt/roam/mp/",
    14: "/opt/roam/mp/",
    15: DATA_PATH+"/input/TEST3-GND/",
    16: DATA_PATH+"/input/TEST3-GND/"
    }

CHASER_TRAJS_ISS_SIM = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"", # test2 target coordinator unit test
    3: DATA_PATH+"/input/TEST3-ISS/", # test3 tube-MPC unit test (and gain switch)
    4: DATA_PATH+"/input/TEST4-ISS/", # Test4 PD attitude control unit test
    5: TRAJ_GEN_PATH+"/bin/iss-traj2-public/", # test5 motion planner unit test
    6: DATA_PATH+"", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/chaser_slam_traj.csv", # test7 SLAM unit test (chaser motion)
    8: TRAJ_GEN_PATH+"/bin/iss-traj2-public/", # test8 full pipeline: nominal MPC, EKF state mode
    9: TRAJ_GEN_PATH+"/bin/iss-traj2-public/", # test9 full pipeline: tube MPC, EKF state mode
    10: TRAJ_GEN_PATH+"/bin/iss-traj2-public/", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: TRAJ_GEN_PATH+"/bin/iss-traj2-public/", # test11 full pipeline: tube MPC, SLAM state mode
    12: DATA_PATH+"/input/TEST12-ISS/", # test12 predefined trajectory tube MPC with Astrobee noise levels
    13: TRAJ_GEN_PATH+"/bin/iss-traj2-public/",
    14: TRAJ_GEN_PATH+"/bin/iss-traj2-public/",
    15: DATA_PATH+"/input/MIT-ISS/", # standard MPC
    16: DATA_PATH+"/input/MIT-ISS/",  # tube MPC
    77: DATA_PATH+"/input/TEST12-ISS/"  # debug
    }

CHASER_TRAJS_ISS_HARDWARE = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"", # test2 target coordinator unit test
    3: DATA_PATH+"/input/TEST3-ISS/", # test3 tube-MPC unit test
    4: DATA_PATH+"/input/TEST4-ISS/", # Test4 PD attitude control unit test
    5: "/opt/roam/mp-iss/", # test5 motion planner unit test
    6: DATA_PATH+"", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/chaser_slam_traj.csv", # test7 SLAM unit test (chaser motion)
    8: "/opt/roam/mp-iss/", # test8 full pipeline: nominal MPC, EKF state mode
    9: "/opt/roam/mp-iss/", # test9 full pipeline: tube MPC, EKF state mode
    10: "/opt/roam/mp-iss/", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: "/opt/roam/mp-iss/", # test11 full pipeline: tube MPC, SLAM state mode
    12: DATA_PATH+"/input/TEST12-ISS/", # test12 predefined trajectory tube MPC with Astrobee noise levels
    13: "/opt/roam/mp-iss/",
    14: "/opt/roam/mp-iss/",
    15: DATA_PATH+"/input/MIT-ISS/", # standard MPC
    16: DATA_PATH+"/input/MIT-ISS/"  # tube MPC
    }

# Target trajectory files, indexed by test#
TARGET_TRAJS_GND_SIM = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test2 target coordinator unit test
    3: DATA_PATH+"", # test3 tube-MPC unit test
    4: DATA_PATH+"", # Test4 PD attitude control unit test
    5: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test5 motion planner unit test
    6: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test7 SLAM unit test (chaser motion)
    8: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test8 full pipeline: nominal MPC, EKF state mode
    9: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test9 full pipeline: tube MPC, EKF state mode
    10: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test11 full pipeline: tube MPC, SLAM state mode
    12: "",  # test12 predefined trajectory tube MPC with Astrobee noise levels
    13: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat",
    14: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat",
    15: "",
    16: ""
    }

TARGET_TRAJS_GND_HARDWARE = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test2 target coordinator unit test
    3: DATA_PATH+"", # test3 tube-MPC unit test
    4: DATA_PATH+"", # Test4 PD attitude control unit test
    5: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test5 motion planner unit test
    6: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test7 SLAM unit test (chaser motion)
    8: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test8 full pipeline: nominal MPC, EKF state mode
    9: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test9 full pipeline: tube MPC, EKF state mode
    10: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat", # test11 full pipeline: tube MPC, SLAM state mode
    12: "",
    13: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat",
    14: DATA_PATH+"/input/sample_trajectories/targ_traj_ground1.dat",
    15: "",
    16: ""
    }

TARGET_TRAJS_ISS_SIM = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test2 target coordinator unit test
    3: DATA_PATH+"", # test3 tube-MPC unit test
    4: DATA_PATH+"", # Test4 PD attitude control unit test
    5: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test5 motion planner unit test
    6: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test7 SLAM unit test (chaser motion)
    8: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test8 full pipeline: nominal MPC, EKF state mode
    9: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test9 full pipeline: tube MPC, EKF state mode
    10: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test11 full pipeline: tube MPC, SLAM state mode
    12: "",
    13: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat",
    14: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat",
    15: "",
    16: ""
    }

TARGET_TRAJS_ISS_HARDWARE = {1: DATA_PATH+"", # test1 software launch checkout
    2: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test2 target coordinator unit test
    3: DATA_PATH+"", # test3 tube-MPC unit test
    4: DATA_PATH+"", # Test4 PD attitude control unit test
    5: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test5 motion planner unit test
    6: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test6 SLAM unit test (chaser stationary)
    7: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test7 SLAM unit test (chaser motion)
    8: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test8 full pipeline: nominal MPC, EKF state mode
    9: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test9 full pipeline: tube MPC, EKF state mode
    10: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test10 full pipeline: tube MPC, EKF state mode, HazCam recording
    11: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # test11 full pipeline: tube MPC, SLAM state mode
    12: "",
    13: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat",
    14: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat",
    15: "",
    16: ""
    }

TARGET_TRAJS_ISS_LUT = {0: DATA_PATH+"/input/sample_trajectories/targ_traj_iss11.dat", # triaxial, x-axis
    1: DATA_PATH+"/input/sample_trajectories/targ_traj_iss12.dat", # flat-spin, x-axis
    2: DATA_PATH+"/input/sample_trajectories/targ_traj_iss13.dat", # stopped, x-axis
    3: DATA_PATH+"/input/sample_trajectories/targ_traj_iss31.dat", # triaxial, z-axis
    4: DATA_PATH+"/input/sample_trajectories/targ_traj_iss32.dat", # flat-spin, z-axis
    5: DATA_PATH+"/input/sample_trajectories/targ_traj_iss33.dat", # stopped, z-axis
    6: DATA_PATH+"/input/sample_trajectories/targ_traj_iss21.dat", # triaxial, y-axis
    7: DATA_PATH+"/input/sample_trajectories/targ_traj_iss22.dat", # flat-spin, y-axis
    8: DATA_PATH+"/input/sample_trajectories/targ_traj_iss23.dat" # stopped, y-axis
    }

CHASER_TRAJ_ISS_LUT_SIM = TRAJ_GEN_PATH+"/bin/iss-traj2-public/"
CHASER_TRAJ_ISS_LUT_HARDWARE = "/opt/roam/mp-iss/"

# a dictionary of all possible trajectory paths for "ground-sim", "ground-hardware", "iss-sim", and "iss-hardware"
CHASER_TRAJS = {"ground":
                    {"sim": CHASER_TRAJS_GND_SIM, "hardware": CHASER_TRAJS_GND_HARDWARE},
                "iss":
                    {"sim": CHASER_TRAJS_ISS_SIM, "hardware": CHASER_TRAJS_ISS_HARDWARE}
                }

TARGET_TRAJS = {"ground":
                    {"sim": TARGET_TRAJS_GND_SIM, "hardware": TARGET_TRAJS_GND_HARDWARE},
                "iss":
                    {"sim": TARGET_TRAJS_ISS_SIM, "hardware": TARGET_TRAJS_ISS_HARDWARE}
                }

class ASAP:
    def __init__(self, bee_roles, bee_topic_prefixes, bee_data_dirs):
        """
        bee_roles : ['role1' ... 'rolen']
        bee_topic_prefixes : [names of astrobees]
        bee_data_dirs : [[test1_dir, ... testn_dir]
        				 [test1_dir, ... testn_dir]] traj information files for each [bee, test]
        bee_is_ground: environment, {'true', 'false'}
        bee_is_sim: {True', False} (as a bool)
        my_role : {'target', 'chaser'}. Uses /robot_name on hardware to set.
        """
        # Data for both Astrobeees
        self.bee_roles = bee_roles
        self.bee_topic_prefixes = bee_topic_prefixes
        self.bee_data_dirs = bee_data_dirs

        # Data specific to each Astrobee instance
        self.test_num = -1
        self.bee_is_ground = 'false'
        self.bee_is_sim = False
        self.my_role = 'chaser'
        self.test_started = False
        self.role_from_GDS = "robot_name"
        self.llp_ip = "10.42.0.4"
        self.bag_proc = []  # process for bag recording
        self.roam_bagger = "enabled"  # {enabled or disabled} is the roam bagger enabled?
        self.bag_robot_name = "queen"

        # GDS telemetry data (same stuff as TDStatus message)
        self.test_LUT = "y"
        self.test_tumble_type = ""
        self.test_control_mode = ""
        self.test_state_mode = ""
        self.LUT_param = ""
        self.motion_planner_interface_activate = False
        self.uc_bound_activate = False
        self.slam_converged = False
        self.inertia_estimated = False
        self.uc_bound_finished = False
        self.mrpi_finished = False
        self.traj_finished = False
        self.test_finished = False
        self.td_flight_mode = "off"
        self.td_control_mode = "inactive"
        self.td_state_mode = "ekf"
        self.casadi_on_target = False
        self.slam_activate = False
        self.chaser_regulate_finished = False
        self.target_regulate_finished = False
        self.motion_plan_wait_time = 0.0
        self.motion_plan_finished = False
        self.default_control = "true"
        # my_role is also sent


    def start_td_nodelets(self):
        """ Start up all TumbleDock nodelets. Works for /queen/ and /bumble/ or "/" namespaces.
        """
        if self.bee_is_ground == "true":
            ENV = "ground"
        else:
            ENV = "iss"

        if ASAP_main.my_role == 'target':
            if ASAP_main.bee_is_sim:
                targ_launch_command = "roslaunch "+TD_TARGET_LAUNCH_PATH
            else:
                targ_launch_command = "roslaunch "+TD_TARGET_LAUNCH_PATH+" llp:="+ASAP_main.llp_ip
            command = targ_launch_command

        elif ASAP_main.my_role == 'chaser':
            if ASAP_main.bee_is_sim:
                chaser_launch_command = "roslaunch "+TD_CHASER_LAUNCH_PATH+" env:="+ENV
            else:
                chaser_launch_command = "roslaunch "+TD_CHASER_LAUNCH_PATH+" env:="+ENV+" llp:="+ASAP_main.llp_ip
            command = chaser_launch_command

        global launch_proc
        launch_proc = subprocess.Popen(command, shell=True)

    def stop_td_nodelets(self):
        """ Stop all TumbleDock nodelets. Works for /queen/ and /bumble/ or / namespaces.
        """
        commands = []
        manager_commands = []

        if ASAP_main.my_role == 'target':
            if ASAP_main.bee_is_sim:
                td_manager_cmd = "rosnode kill /bumble/td_mlp_manager"
                targ_coord_cmd = "rosnode kill /bumble/target_coordinator"
                casadi_cmd = "rosnode kill /bumble/casadi_nmpc"
            else:
                td_manager_cmd = "rosnode kill /td_mlp_manager"
                targ_coord_cmd = "rosnode kill /target_coordinator"
                casadi_cmd = "rosnode kill /casadi_nmpc"
            manager_commands = [td_manager_cmd]
            commands = [targ_coord_cmd, casadi_cmd]
        elif ASAP_main.my_role == 'chaser':
            if ASAP_main.bee_is_sim:
                td_llp_manager_cmd = "rosnode kill /queen/td_llp_manager"
                td_mlp_manager_cmd = "rosnode kill /queen/td_mlp_manager"
                chaser_coord_cmd = "rosnode kill /queen/chaser_coordinator"
                casadi_cmd = "rosnode kill /queen/casadi_nmpc"
                slam_cmd = "rosnode kill /queen/mit_slam"
                uc_cmd = "rosnode kill /queen/uc_bound"
                traj_cmd = "rosnode kill /queen/motion_planner_interface"
                uc_dummy_cmd = "rosnode kill /queen/uc_bound_dummy_est"
                z_poly_cmd = "rosnode kill /queen/z_poly_calc"
            else:
                td_llp_manager_cmd = "rosnode kill /td_llp_manager"
                td_mlp_manager_cmd = "rosnode kill /td_mlp_manager"
                chaser_coord_cmd = "rosnode kill /chaser_coordinator"
                casadi_cmd = "rosnode kill /casadi_nmpc"
                slam_cmd = "rosnode kill /mit_slam"
                uc_cmd = "rosnode kill /uc_bound"
                traj_cmd = "rosnode kill /motion_planner_interface"
                uc_dummy_cmd = "rosnode kill /uc_bound_dummy_est"
                z_poly_cmd = "rosnode kill /z_poly_calc"
            manager_commands = [td_llp_manager_cmd, td_mlp_manager_cmd]
            commands = [chaser_coord_cmd, casadi_cmd, slam_cmd, uc_cmd, traj_cmd, uc_dummy_cmd, z_poly_cmd]

        # Use ROS to kill processes
        processes = []
        for command in commands:
            p = subprocess.Popen(command, shell=True)
            processes.append(p)
        time.sleep(5)
        for p in processes:
            p.kill()

    def rosbag_start(self, test_number):
        """
        Start rosbag recording.
        """
        global ROSBAG_NAME

        BAG_PATH = self.get_bag_path()

        epoch_str = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        ROSBAG_NAME = epoch_str + "_roam_test" + str(test_number)
        #epoch_str = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        #ROSBAG_NAME = "test" + str(test_number) + "_" + epoch_str
        print("rosbag started: " + ROSBAG_NAME)

        # Starts a bag for either local machine or hardware.
        command = ""

        # choose target topics, no haz cam chaser topics, or full chaser topics
        if self.bee_is_sim:  # sim
            if (self.my_role == 'target'):
                if (test_number == 2):
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_target.bag " + TOPICS_SIM_TARG_NAVCAM + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_target.bag " + TOPICS_SIM_TARG + " __name:=roam_bagger"
            else:
                if (test_number == 6 or test_number == 7 or test_number == 10):
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_chaser_w_haz.bag " + TOPICS_SIM_CHASER_FULL + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_chaser.bag " + TOPICS_SIM_CHASER_NOHAZ + " __name:=roam_bagger"
        else:  # hardware
            if (self.my_role == 'target'):
                if (test_number == 2):
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_target.bag " + TOPICS_HARDWARE_TARG_NAVCAM + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_target.bag " + TOPICS_HARDWARE_TARG + " __name:=roam_bagger"
            else:
                if (test_number == 6 or test_number == 7 or test_number == 10):
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_chaser_w_haz.bag " + TOPICS_HARDWARE_CHASER_FULL + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O "+ BAG_PATH + ROSBAG_NAME + "_chaser.bag " + TOPICS_HARDWARE_CHASER_NOHAZ + " __name:=roam_bagger"

        command = shlex.split(command)
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)  # so SIGINT is caught by the parent

    def rosbag_stop(self):
        """
        Kill rosbag on ctl-c or -1.
        """
        # self.bag_proc.send_signal(subprocess.signal.SIGINT)
        print('\n[EXECUTE_ASAP]: SIGINT or CTRL-C detected, checking for rosbags...')
        try:
            subprocess.check_call(['rosnode', 'kill', 'roam_bagger'])
            print("[EXECUTE_ASAP]: ...rosbag killed.")
        except:
            print("[EXECUTE_ASAP]: ...no rosbag started.")

    def handler(self, signal_received, frame):
        """
        Catch ctl-c.
        """
        self.rosbag_stop()
        sys.exit(0)

    def bee_name_callback(self, name_data):
        """ Only used for hardware. Reads in role from /robot_name.
        * Roles are HARDCODED, capitalization matters.
        * Bypassed if self.gds_role is not robot_name.
        * Also sets bag robot name (must be lowercase).
        """
        # for bagger
        if name_data.data == "Honey":
            self.bag_robot_name = "honey"
        elif name_data.data == "Bsharp":
            self.bag_robot_name = "bsharp"
        elif name_data.data == "Wannabee":
            self.bag_robot_name = "wannabee"
        elif name_data.data == "Bumble":
            self.bag_robot_name = "bumble"
        elif name_data.data == "Queen":
            self.bag_robot_name = "queen"

        # for role
        if self.gds_role == "robot_name":  # if role has not been set by GDS
            if name_data.data == "Honey" or name_data.data == "Queen" or name_data.data == "Bsharp":
                self.my_role = 'chaser'
            else:
                self.my_role = 'target'

    def status_callback(self, status_msg):
        """ Subscribe to status published by C++ coordinator.
        Used to check to make sure flight mode is set to off and control is
        default before killing C++ nodelets.
        """
        if status_msg.default_control:
            self.default_control = 'true'
        else:
            self.default_control = 'false'
        self.td_flight_mode = status_msg.td_flight_mode
        self.motion_plan_finished = status_msg.motion_plan_finished
        self.td_control_mode = status_msg.td_control_mode
        self.motion_plan_wait_time = status_msg.motion_plan_wait_time
        self.chaser_regulate_finished = status_msg.chaser_regulate_finished
        self.target_regulate_finished = status_msg.target_regulate_finished
        self.slam_activate = status_msg.slam_activate

        self.test_LUT = status_msg.test_LUT
        self.test_tumble_type = status_msg.test_tumble_type
        self.test_control_mode = status_msg.test_control_mode
        self.test_state_mode = status_msg.test_state_mode
        self.LUT_param = status_msg.LUT_param
        self.motion_planner_interface_activate = status_msg.motion_planner_interface_activate
        self.uc_bound_activate = status_msg.uc_bound_activate
        self.slam_converged = status_msg.slam_converged
        self.inertia_estimated = status_msg.inertia_estimated
        self.uc_bound_finished = status_msg.uc_bound_finished
        self.mrpi_finished = status_msg.mrpi_finished
        self.traj_finished = status_msg.traj_finished
        self.test_finished = status_msg.test_finished
        self.td_flight_mode = status_msg.td_flight_mode
        self.td_control_mode = status_msg.td_control_mode
        self.td_state_mode = status_msg.td_state_mode
        self.casadi_on_target = status_msg.casadi_on_target

    def get_bag_path(self):
        """ Gives the rosbag data path.
        """
        global BAG_PATH_HARDWARE
        global BAG_PATH_SIM

        epoch_str = datetime.datetime.now().strftime("%Y-%m-%d")

        if self.bee_is_sim == True:
            #BAG_PATH = BAG_PATH_SIM
            BAG_PATH = DATA_PATH + "/output/rosbags/data/bags/" + epoch_str + "/" + self.bag_robot_name + "/delayed/"
        else:
            #BAG_PATH = BAG_PATH_HARDWARE
            BAG_PATH = "/data/bags/" + epoch_str + "/" + self.bag_robot_name + "/delayed/"
        return BAG_PATH

    def run_test(self):
        """ Runs a ROAM test, either for 'chaser' or 'target'
        """
        self.test_started = True
        test_number = self.test_num

        if self.bee_is_ground == "true":
            ENV = "ground"
        else:
            ENV = "iss"

        if self.bee_is_sim == True:
            SIM = "sim"
        else:
            SIM = "hardware"
        print("[EXECUTE_ASAP]: Beginning test " + str(test_number) + "...")

        # Start TD nodelets, start BOTH chaser_asap and target_asap threads since it's sim.
        if self.bee_is_sim == True:
            print("[EXECUTE_ASAP]: ENV:= "+ENV+"; SIM:= "+SIM)
            rospy.set_param("/td/test_killed", False)

            # Begin ROS data recording
            if test_number != -1 and self.roam_bagger == "enabled":
                self.rosbag_start(test_number)

            # Start chaser_asap and target_asap threads
            if self.my_role == 'target':
                # Run target_asap for test parameters
                traj_file = ""
                if (test_number >= 100):
                    if str(test_number)[0] == '1' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[0]
                    elif str(test_number)[0] == '1' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[1]
                    elif str(test_number)[0] == '1' and str(test_number)[1] == '3':
                        traj_file = TARGET_TRAJS_ISS_LUT[2]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[3]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[4]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '3':
		        traj_file = TARGET_TRAJS_ISS_LUT[5]
		    elif str(test_number)[0] == '2' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[6]
                    elif str(test_number)[0] == '2' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[7]
                    else:
                        traj_file = TARGET_TRAJS_ISS_LUT[8]
                else:
                    traj_file = self.bee_data_dirs[0][ENV][SIM][test_number]
                target_asap.target_execute_test(traj_file, self.bee_topic_prefixes[0], test_number, self.bee_is_ground, True)

                # Ensure params are set before nodelets start running
                time.sleep(2)

                print("[EXECUTE_ASAP]: Launching nodelets.")
                # Launch nodelets
                self.start_td_nodelets()

                # target coordinator takes it from here
                print("[EXECUTE_ASAP]: Test passed to target coordinator.")

            elif self.my_role == 'chaser':
                # Run chaser_asap for test parameters
                if (test_number >= 100):
                    traj_file = CHASER_TRAJ_ISS_LUT_SIM
                else:
                    traj_file = self.bee_data_dirs[1][ENV][SIM][test_number]
                chaser_asap.chaser_execute_test(traj_file, self.bee_topic_prefixes[1], test_number, self.bee_is_ground, True)

                # Ensure params are set before nodelets start running
                time.sleep(2)

                # Launch nodelets
                self.start_td_nodelets()

                # target coordinator takes it from here
                print("[EXECUTE_ASAP]: Test passed to chaser coordinator.")

    	# Assume this is running individually on each Astrobee. Run a single thread.
        else:
            print("*** [EXECUTE_ASAP]: Using ENV:= "+ENV+" SIM:= "+SIM+" ***")

            # Begin ROS data recording
            if test_number != -1 and self.roam_bagger == "enabled":
                self.rosbag_start(test_number)

            if self.my_role == 'target':
                # Run target_asap for test parameters
                if (test_number >= 100):
                    if str(test_number)[0] == '1' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[0]
                    elif str(test_number)[0] == '1' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[1]
                    elif str(test_number)[0] == '1' and str(test_number)[1] == '3':
                        traj_file = TARGET_TRAJS_ISS_LUT[2]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[3]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[4]
                    elif str(test_number)[0] == '3' and str(test_number)[1] == '3':
		                traj_file = TARGET_TRAJS_ISS_LUT[5]
                    elif str(test_number)[0] == '2' and str(test_number)[1] == '1':
                        traj_file = TARGET_TRAJS_ISS_LUT[6]
                    elif str(test_number)[0] == '2' and str(test_number)[1] == '2':
                        traj_file = TARGET_TRAJS_ISS_LUT[7]
                    else:
                        traj_file = TARGET_TRAJS_ISS_LUT[8]
                else:
                    traj_file = self.bee_data_dirs[0][ENV][SIM][test_number]
                    print(traj_file)
                target_asap.target_execute_test(traj_file, self.bee_topic_prefixes[0], test_number, self.bee_is_ground, False)

                # Ensure params are set before nodelets start running
                time.sleep(2)

                self.start_td_nodelets()

                # target coordinator takes it from here
                print("[EXECUTE_ASAP]: Test passed to chaser coordinator.")

            elif self.my_role == 'chaser':
                # Run chaser_asap for test parameters
                if (test_number >= 100):
                    traj_file = CHASER_TRAJ_ISS_LUT_HARDWARE
                else:
                    traj_file = self.bee_data_dirs[1][ENV][SIM][test_number]
                chaser_asap.chaser_execute_test(traj_file, self.bee_topic_prefixes[1], test_number, self.bee_is_ground, False)

                # Ensure params are set before nodelets start running
                time.sleep(2)

                # Launch nodelets
                self.start_td_nodelets()

                # target coordinator takes it from here
                print("[EXECUTE_ASAP]: Test passed to chaser coordinator.")

    def stop_test(self):
        # Stop the test.
        self.test_started = False
        rospy.set_param("/td/test_killed", True)
        print('[EXECUTE_ASAP]: Test canceled.')

        # Stop ROS bag recording
        print('[EXECUTE_ASAP]: Killing ROS bag.')
        self.rosbag_stop()

        # Wait to make sure flight mode is off and controller is default before
        # killing TD nodelets.
        t0 = rospy.get_time()
        while self.default_control != "true" or self.td_flight_mode != "off":
            print('[EXECUTE_ASAP]: Waiting for coordinator to re-enable default control and set flight mode to off.')
            print(self.td_flight_mode)
            print(self.default_control)
            time.sleep(1)
            if (rospy.get_time() - t0 > 10.0):
                break

        # Kill TD nodelets
        print('[EXECUTE_ASAP]: Killing TD nodelets.')
        self.stop_td_nodelets()
        print('[EXECUTE_ASAP]: TD nodelets killed.')

        # Turn off any signal signal_light_state
        msg=SignalState.STOP_ALL_LIGHTS

    def test_num_okay(self):
        """ Only allow valid test numbers."""
        test_num = self.test_num
        str_test_num = str(test_num)
        OKAY = False

        # unit tests (1-12)
        if (test_num >= 1) and (test_num <=16) or test_num == 77:
            OKAY = True

        # standard test
        if len(str_test_num) == 8:  # check digit-by-digit
            if (int(str_test_num[0]) >= 1) and (int(str_test_num[0]) <= 3) and \
               (int(str_test_num[1]) >= 1) and (int(str_test_num[1]) <= 3) and \
               (int(str_test_num[2]) >= 1) and (int(str_test_num[2]) <= 2) and \
               (int(str_test_num[3]) >= 1) and (int(str_test_num[3]) <= 4) and \
               (int(str_test_num[4]) >= 1) and (int(str_test_num[4]) <= 2) and \
               (int(str_test_num[5]) >= 1) and (int(str_test_num[5]) <= 2) and \
               (int(str_test_num[6]) >= 1) and (int(str_test_num[6]) <= 2) and \
               (int(str_test_num[7]) >= 1) and (int(str_test_num[7]) <= 2):
               OKAY = True

        # stop test
        if test_num == -1:
           OKAY = True
        print(OKAY)
        return OKAY

    def stop_signal_lights(self, pub_signal_lights):
        """ Stop the signal lights on call.
        """
        try:
            state = SignalState.STOP_ALL_LIGHTS
            msg = SignalState()
            msg.state = state
            pub_signal_lights.publish(msg)
        except:
            print("Unable to stop signal lights.")

"""
Run on node startup. Waits for GDS parameters to be set before proceeding.

If SIM, then ROBOT_NAME_ARG gets set as a launchfile commandline option to specify role.
"""
if __name__ == "__main__":
    # initialize node
    rospy.init_node('execute_asap', anonymous=True)

    # This is a special comparison that only works if a launch file with the correct number of arguments is used.
    myargv = rospy.myargv(argv=sys.argv)
    if(len(myargv) != 3):  # happens on hardware
        ROBOT_NAME_ARG = "/"
    else:  # happens in sim
        ROBOT_NAME_ARG = myargv[1]  # robot name as argument for sim
    print("[EXECUTE_ASAP]: " + ROBOT_NAME_ARG)

    # Set up the main testing interface.
    ASAP_main = ASAP(bee_roles =          ['target',     'chaser'],
                     bee_topic_prefixes = ['/bumble/',   '/queen/'],
                     bee_data_dirs =      [TARGET_TRAJS, CHASER_TRAJS])

    signal(SIGINT, ASAP_main.handler)  # for ctl-c handling

    # for hardware
    test_number_msg_name = "/td/test_number"
    status_msg_name = "/td/status"
    signal_msg_name = "/signals"

    # for sim
    if ROBOT_NAME_ARG == "queen":
        ASAP_main.my_role = "chaser"
        ASAP_main.bag_robot_name = "queen"
        test_number_msg_name = "/queen/td/test_number"
        status_msg_name = "/queen/td/status"
        signal_msg_name = "/queen/signals"
    elif ROBOT_NAME_ARG == "bumble":
        ASAP_main.my_role = "target"
        ASAP_main.bag_robot_name = "bumble"
        test_number_msg_name = "/bumble/td/test_number"
        status_msg_name = "/bumble/td/status"
        signal_msg_name = "bumble/signals"

    ### ROS initialization
    # initialize GDS params
    rospy.set_param("/td/gds_ground", "false")
    rospy.set_param("/td/gds_sim", "hardware")
    rospy.set_param("/td/gds_test_num", -1)
    rospy.set_param("/td/role_from_GDS", "robot_name")  # default to using robot_name
    rospy.set_param("/td/gds_roam_bagger", "enabled")  # "true" or "false"

    # subscribers
    rospy.Subscriber("/robot_name", String, ASAP_main.bee_name_callback)  # subscriber for robot name topic
    rospy.Subscriber(status_msg_name, TDStatus, ASAP_main.status_callback)  # subscriber for TD status message (published by coordinators)

    # publisher for test_number
    pub_test_number = rospy.Publisher(test_number_msg_name, TDTestNumber, queue_size=10)
    pub_signal =rospy.Publisher(signal_msg_name, SignalState, queue_size=1, latch=True)

    ### ROS loop for checking test number from GDS, publishing test number, subscribers for robot name/TD status message
    ASAP_RATE = 0.5
    param_set_count = 0
    global_gds_param_count = 0
    sleep_rate = rospy.Rate(ASAP_RATE)
    while not rospy.is_shutdown():
        # check GDS params
        ground = rospy.get_param("/td/gds_ground")
        sim_string = rospy.get_param("/td/gds_sim")
        test_num = rospy.get_param("/td/gds_test_num")
        role_from_GDS = rospy.get_param("/td/role_from_GDS")  # overrides /robot_name if set!
        roam_bagger = rospy.get_param("/td/gds_roam_bagger")  # force to be string

        # set ASAP based on GDS
        ASAP_main.bee_is_ground = ground
        if sim_string == "sim":
            rospy.set_param('/td/sim', 'true')  # crucial for correct motion planner call
            ASAP_main.bee_is_sim = True
        else:
            rospy.set_param('/td/sim', 'false')
            ASAP_main.bee_is_sim = False
        ASAP_main.test_num = test_num
        ASAP_main.role_from_GDS = role_from_GDS
        if role_from_GDS != "robot_name":  # if not robot_name
            ASAP_main.my_role = role_from_GDS
        ASAP_main.roam_bagger = roam_bagger

        # Publish test test_number
        msg = TDTestNumber()
        msg.stamp = rospy.get_rostime()
        msg.test_number = ASAP_main.test_num
        msg.role = ASAP_main.my_role
        pub_test_number.publish(msg)

        # Set params to send GDS telemetry (5x slower)
        param_set_count += 1
        if (param_set_count > 5):
            param_set_count = 0
            rospy.set_param("/td/chaser/gds_telem",
                [str(global_gds_param_count), str(ASAP_main.test_num), str(ASAP_main.td_flight_mode), str(ASAP_main.td_control_mode), str(ASAP_main.slam_activate), str(ASAP_main.target_regulate_finished),
                 str(ASAP_main.chaser_regulate_finished), str(ASAP_main.motion_plan_finished), str(ASAP_main.motion_plan_wait_time), str(ASAP_main.default_control),
                 str(ASAP_main.my_role), str(ASAP_main.test_LUT), str(ASAP_main.test_tumble_type), str(ASAP_main.test_control_mode), str(ASAP_main.test_state_mode),
                 str(ASAP_main.LUT_param), str(ASAP_main.motion_planner_interface_activate), str(ASAP_main.uc_bound_activate), str(ASAP_main.slam_converged), str(ASAP_main.inertia_estimated),
                 str(ASAP_main.uc_bound_finished), str(ASAP_main.mrpi_finished), str(ASAP_main.traj_finished), str(ASAP_main.test_finished), str(ASAP_main.td_state_mode), str(ASAP_main.casadi_on_target)])
            global_gds_param_count = global_gds_param_count + 1

        # Run/stop tests or wait based on GDS params. Start if not -1.
        if (ASAP_main.test_num is not -1 and ASAP_main.test_started == False):
            if ASAP_main.test_num_okay():  # sanity check the test num before sending
                ASAP_main.run_test()
            else:
                ASAP_main.test_num = -1
        if (ASAP_main.test_num == -1 and ASAP_main.test_started == True):
            ASAP_main.stop_test()
            ASAP_main.stop_signal_lights(pub_signal)

        sleep_rate.sleep()
