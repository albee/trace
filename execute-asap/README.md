# execute_asap

*Note: MIT has open-sourced an updated version of `execute_asap`. However, this version is compatible with TRACE testing.*

A set of tools that handle high-level coordination of ROAM-TS1 and ROAM-TS2. Trajectory input can be in the form of a CSV or standard formatted directory (see `data/input/sample-trajectories`).

- `execute_asap.py` : high-level coordinating script
- `chaser_test.py` : chaser-level coordinating script
- `target_test.py` : target-level coordinating script

The `execute_asap` nodelet listens to incoming GDS ROS parameters that define a test. The parameters are 1) whether the test is being run in simulation are on hardware, 2) whether the test is on the ISS or on the ground, and 3) the test number. For now, these parameters are defined as

- `/td/gds_sim`  (String)
- `/td/gds_ground`   (String)
- `/td/gds_test_num`   (int)

## Usage:

For running a test in the simulator, use your desired TEST_NUMBER as an argument with the `pub_gds_topics.py` script. This script sets the parameters that GDS eventually will set. `execute_asap.py` listens to these parameters, launches the TRACE nodelets, and hands it off to individual *roles*, in this case
Chaser and Target. `chaser_asap.py` and `target_asap.py` set the chaser/target initial conditions and test-specific parameters; real-time logic throughout a test is handled by *coordinator nodelets*, `chaser_coordinator_nodelet.cc` and `target_coordinator_nodelet.cc`.

Run using: rosrun execute_asap pub_gds_topics.py [--ground] [--sim] test_number, e.g.,

`rosrun execute_asap pub_gds_topics.py --ground --sim 1` : to run test 1 on the ground for sim

`[-g, --ground]` : Run a ground test. Default to ISS.

`[-s, --sim]` : Run a simulation test. Defaults to hardware.

## Canceling a test:

To end a test and kill TRACE nodelets, run the script with a -1:

`rosrun execute_asap pub_gds_topics.py --ground --sim -1` : to cancel tests on the ground for sim.

This will stop all ROS bag recording and will also kill the TRACE nodelets. The `execute_asap` nodelet will still run, offering the opportunity to run another test and launch TRACE nodelets again.

## Robot namespacing:

For the simulator, each robot defines its role in `execute_asap.py` via a robot name argument that is passed in the `execute_asap` node launch process. For hardware, `execute_asap` subscribes to the topic `/robot_name` to define its role. If the robot name is `bumble`, it is the target. If the robot name is `honey`, it is the chaser.

## Topic recording:

Specify topics to record with `TOPICS`. Name the rosbag with `ROSBAG_NAME`. TODO: finalize data topics, have script to convert to .csv files and plot.

## Pre-set trajectories:

Specify trajectory paths with the `*_TRAJS_*` variables.

## Simulator start-up:

Start up the Astrobee sim with '/bumble' and '/honey' for the ISS:
`roslaunch astrobee sim_td.launch`

You can set additional parameters on what to launch in sim_td.launch and in td_MLP.launch/td_LLP.launch (both in the `develop/execute_asap/td_launch_files` folder). Also, make sure parameters in target_asap.py and chaser_asap.py are set as desired.

## Adding Tests

You can add tests under `*_execute_test` in `chaser_asap.py` and `target_asap.py`, making sure that individual nodelets are able to react to any parameters sent out.
All commanding should occur via the `/honey/td/instruct/` and `/td/instruct` parameters, which command *chunks* for each coordinator to execute.

---

## Notes
Note for logging:
logging levels are set in astrobee/resources/logging/config
