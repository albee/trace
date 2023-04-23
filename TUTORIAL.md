# Tutorial

This walkthrough overviews the code for TRACE, which includes a set of rendezvous nodelets, Python coordinating scripts, standalone Python scripts, and a data storage folder. TRACE is intergrated with the Astrobee simulation for testing purposes, but can be adapted to other simulation environments. This tutorial assumes
you've successfully set up your environment from README.md.


## Simulation Usage

- Launch environment (ISS):

`roslaunch astrobee sim_td.launch world:=iss rviz:=true`

- Launch ground environment (ground):

`roslaunch astrobee sim_td.launch world:=granite rviz:=true`

- When switching between environments be sure to reset accelerometer bias for both Astrobees:

`rosrun executive teleop_tool -ns "bumble/" -reset_bias`
`rosrun executive teleop_tool -ns "honey/" -reset_bias`

- Run a TRACE test (after sim launch)
```
`rosrun execute_asap pub_gds_topics.py --ground --sim 1` : to run test 1 on the ground for sim

`[-g, --ground]` : Run a ground test. Default to ISS.

`[-s, --sim]` : Run a simulation test. Defaults to hardware.
```

See `execute_asap/README.md` for high-level usage instructions to understand how nodes are started/stoppped and the execution flow.

## Test List

**Unit Tests (all unit tests are in the y-axis direction and use triaxial tumble):** 
- -1 stop and reset nodelets
- 1 Quick Checkout: Software launch checkout (no motion or impeller start-up)
- 2 Target tumble unit test (target motion only)
- 3 Tube-MPC unit test with gain switch/characterization (chaser motion only)
- 4 PD Attitude control (chaser motion only)
- 5 Motion planner unit test (chaser motion only)
- 6 SLAM stationary unit test (target motion only, chaser stationary)
- 7 SLAM motion unit test (both motion)
- 8 Full pipeline: Nominal MPC, EKF state mode
- 9 Full pipeline: Tube MPC, EKF state mode
- 10 Full pipeline: Tube MPC, EKF state mode, recording HazCam data
- ~~11 Full pipeline Tube MPC, SLAM state mode, online updates~~
- 12 Tube MPC full traj with Astrobee noise uc_bound
- 13 Full pipeline: SLAM spoof: test8 w/ SLAM spoof (EKF state mode, no online updates)
- ~~14 Full pipeline: SLAM spoof: test11 w/ SLAM spoof (SLAM state mode, online updates)~~
- 15: Multi-waypoint translation test with Astrobee noise uc_bound, standard MPC
- 16: Multi-waypoint translation test with Astrobee noise uc_bound, tube MPC
 
**Standard Tests/Full Pipeline**
Consult Notion for latest info on test parameters. `x` indicates different parameter settings for this test.

- 11(x): x-axis, tri-axial tumble
- 12(x): ...
- 13(x)
- 21(x)
- 22(x)
- 23(x)
- 31(x)
- 32(x)...
- 33(x): z-axis, no tumble

# Directory Listing

## backup_controller

A backup PID controller (translational+attitude) for the Astrobee rigid body dynamics (Ian Hokaj). Commit in progress.

## casadi_nmpc

A robust tube MPC controller. Communicates with chaser_coordinator to receive plans, provides output to FAM. Interruptable (implementation in progress). CasADi must be installed! See the README.md.

## chaser_coordinator

The nodelet package for sending trajectory commands to the controller topic (Chaser). Input can be a trajectory file, or a direct topic call with trajectory info (in progress).

## data

data folder for test `input/` and `output/`

## execute_ASAP

High-level commanding scripts. These are currently pure Python scripts that use the ROS parameter server.

## mit_slam

Utilizes GTSAM for smoothing and mapping, Teaser++ for point cloud registration. Teaser++ is automatically built with this repo and shouldn't need extra commands to install. It is required to install GTSAM before compiling this repo, see mit_slam/README.md for more details on this.

## polhode_alignment

Polhode alignment node which performs Setterfield's algorithm to determine principal axis alignment.

## spheres-vertigo

Legacy code to support polhode-alignment and mit-slam.

## target_coordinator

The nodelet package for sending trajectory commands to the controller topic (Target). Input can be a trajectory file, or a direct topic call with trajectory info (in progress).

## trace_msgs

Custom msg definitions used by TRACE nodes.

## motion_planner_interface

An interface to output of a motion planner. Demonstration motion plans are available in `data/input/sample-trajectories` for demonstration purposes.

## uc_bound

Uncertainty characterizer, produces an uncertainty bound used by the Tube MPC.
