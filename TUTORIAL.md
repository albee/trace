# Tutorial

This walkthrough overviews the code for TRACE, which includes a set of nodelets for performing autonomous rendezvous, supporting infrastructure, and integration with the Astorbee simulation environment. While TRACE is out-of-the-box integrated with the Astrobee simulation for testing purposes, it can be adapted to other simulation environments. This tutorial assumes you've successfully set up your environment from README.md.

## Astrobee Simulation Setup

Next, check out the latest tested version of Astrobee's simulation environment and add it to your `trace-ws`:

```bash
export ASTROBEE_WS=${HOME}/trace-ws/
git clone https://github.com/nasa/astrobee.git $ASTROBEE_WS/src/astrobee
pushd $ASTROBEE_WS/src/astrobee
git checkout v0.16.1
git submodule update --init --depth 1 description/media
popd
```

Now, build Astrobee's dependencies:

```bash
sudo apt update
sudo apt upgrade

pushd $ASTROBEE_WS
cd src/astrobee/scripts/setup
./add_ros_repository.sh
sudo apt-get update
cd debians
./build_install_debians.sh
cd ../
./install_desktop_packages.sh
sudo rosdep init
rosdep update
popd
```

Finally, configure and run catkin to build both the Astrobee sim and TRACE:

```bash
pushd $ASTROBEE_WS
./src/astrobee/scripts/configure.sh -l -F -D

# This step is very important! If your paths are wrong, you will not be able to build Astrobee packages.
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+"$CMAKE_PREFIX_PATH:"}${ASTROBEE_WS}/src/astrobee/cmake"
catkin build -j2
```

Don't forget to source your workspace!

```bash
source $ASTROBEE_WS/devel/setup.bash
```


## Simulation Usage

You can launch TRACE in the Astrobee simulation environment using `trace_astrobee_interface`:

- Launch space environment (ISS):

`roslaunch trace_astrobee_interface sim_trace.launch world:=iss rviz:=true`

- Launch ground environment (ground):

`roslaunch trace_astrobee_interface sim_trace.launch world:=granite rviz:=true`

- When switching between environments be sure to reset accelerometer bias for both Astrobees:

`rosrun executive teleop_tool -ns "bumble/" -reset_bias`
`rosrun executive teleop_tool -ns "honey/" -reset_bias`

- Run a TRACE test (after sim launch)

```
`rosrun execute_asap pub_gds_topics.py --sim 1` : to run test 1 on the ISS for sim.

`rosrun execute_asap pub_gds_topics.py --ground --sim 1` : to run test 1 on the ground for sim.

`rosrun execute_asap pub_gds_topics.py --sim -1` : to stop a test. You will see lots of output as the nodes are stopped.


`[-g, --ground]` : Run a ground test. Default to ISS.

`[-s, --sim]` : Run a simulation test. Defaults to hardware.
```

Test2, Test6, and Test7 are good demonstration tests to try out next.

See `execute_asap/README.md` for high-level usage instructions to understand how nodes are started/stopped
and the execution flow of test commands.

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
- 11 (not recommended) Full pipeline Tube MPC, SLAM state mode, online updates
- 12 Tube MPC full traj with Astrobee noise uc_bound
- 13 Full pipeline: SLAM spoof: test8 w/ SLAM spoof (EKF state mode, no online updates)
- 14 (not recommended) Full pipeline: SLAM spoof: test11 w/ SLAM spoof (SLAM state mode, online updates)
- 15: Multi-waypoint translation test with Astrobee noise uc_bound, standard MPC
- 16: Multi-waypoint translation test with Astrobee noise uc_bound, tube MPC
 
**Standard Tests/Full Pipeline**
8-digit tests use parameters to execute the full TRACE pipeline in different ways. `x` indicates different parameter settings for this test.

Currently implemented parameters are:

| Parameter Setting | Direction1x(xxxxxx) | Tumblex1(xxxxxx) | Controllerxx(1xxxxx) | Gainsxx(x1xxxx) | Plannerxx(xx1xxx) | Use Default PD on Target xx(xxx1xx) | Spoofingxx(xxxx1x) | Online Updates xx(xxxxx1) |
|-------------------|--------------------|------------------|----------------------|-----------------|--------------------|------------------------------------|--------------------|--------------------------|
| 1                 | X-axis motion      | Tri-axial tumble        | Standard MPC (Chaser)| Default gains      | Default            | Use MPC (Target)                  | No spoof           | No online updates        |
| 2                 | Y-axis motion      | Near flat spin   | Tube MPC (Chaser)    | Permissive inputs    | Shortest wait time | Use default PD (Target)           | Spoof              | Online updates           |
| 3                 | Z-axis motion      | No tumble        |                      | Tight inputs         |                    |                                    |                    |                          |
| 4                 |                    |                  | "Cautious" tube MPC   |                  |                    | Note: only allowed for tri-axial! | Note: only allowed for y-axis! |                          |

For example, direction and tumble are controlled by the following parameter combinations:

- 11(...): x-axis, tri-axial tumble
- 12(...): ...
- 13(...)
- 21(...)
- 22(...)
- 23(...)
- 31(...)
- 32(...)...
- 33(...): z-axis, no tumble

# Directory Listing

## backup-controller

A backup PID controller (translational+attitude) for the satellite rigid body dynamics.

## casadi-nmpc

A robust tube MPC controller, using CasADi. Communicates with chaser_coordinator to receive plans, provides output to force allocation module. Relies on uc_bound's calculation of robustly safe sets.

## chaser-coordinator

The nodelet package for sending trajectory commands to the controller (Chaser satellite) and handling overall Chaser test operation. Commanded
by execute-asap.

## data

data folder for test `input/` and `output/`. Also contains utilities for interfacing with the motion planner.

## execute-asap

High-level commanding scripts. Pure Python scripts that send off node start-up/shutdown.

## mit-slam

Target tumble characterization. Utilizes GTSAM for smoothing and mapping, Teaser++ for point cloud registration frontend.

## motion-planner-interface

An interface to output of a motion planner. Demonstration motion plans are available in `data/input/sample-trajectories` for demonstration purposes.

## polhode-alignment

Polhode alignment node which performs Setterfield's algorithm to determine principal axis alignment of the Target satellite.

## spheres-vertigo

Legacy code to support `polhode-alignment` and `mit-slam`.

## target-coordinator

The nodelet package for sending trajectory commands to the controller (Target satellite) and handling overall Target test operation. Commanded
by execute-asap.

## trace-msgs

Custom msg definitions used by TRACE nodes.

## uc-bound

Uncertainty characterizer, produces an uncertainty bound used by the casadi-nmpc's robust tube MPC for safe Chaser trajectory tracking.
