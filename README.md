# TRACE

© 2023 Massachusetts Institute of Technology

* ***Latest Known Flight Software Compatbility: v16.1***
* ***Stable branch: `main`***

<p align="center">
  <img src="astrobee_td.gif" />
</p>

TRACE (Tumbling Rendezvous via Autonmous Characterization and Execution) is a set of software modules that enable autonomous rendezvous with arbitrary tumbling targets. A significant percentage of space debris like rocket bodies and [defunct satellites](https://earth.esa.int/eogateway/missions/envisat) are in uncontrolled tumbles, often with unknown tumble characteristics. In order to soft dock for servicing, refueling, or de-orbit, a servicer spacecaft must figure out the nature of the target's tumble and safely reach the target. This often eliminates teleoperation as a feasible option---this is where TRACE comes in. TRACE has been [demonstrated on-orbit](https://spectrum.ieee.org/space-junk-astrobee) over multiple test sessions on the International Space Station using NASA's Astrobee robots. 

## Background
TRACE was developed as part of the ROAM/TumbleDock project, a collaboration between the MIT Space Systems Lab and DLR's Institute of Robotics and Mechatronics. TRACE observes a tumbling Target, estimates its state/parameters, creates a motion plan to reach it, and then plans/tracks robustly in real-time to reach a safe rendezvous position known as the "mating point." This code has flown multiple times on the International Space Station, verifying the real-time performance of the various algorithmic components on hardware. The work, including its assumptions and experimental results, is summarized in detail in few useful references below:

- [K. Albee et al., “A robust observation, planning, and control pipeline for autonomous rendezvous with tumbling targets,” Frontiers in Robotics and AI, vol. 8, p. 234, 2021, doi: 10.3389/frobt.2021.641338.](https://www.frontiersin.org/articles/10.3389/frobt.2021.641338/full)
- [K. Albee, “Online information-aware motion planning with model improvement for uncertain mobile robotics,” Massachusetts Institute of Technology, 2022.](https://dspace.mit.edu/handle/1721.1/144796)
- [C. Oestreich, A. Teran, J. Todd, K. Albee, and R. Linares, “On-orbit inspection of an unknown, tumbling target using NASA’s Astrobee robotic free-flyers,” presented at the Conference on Computer Vision and Pattern Recognition, Virtual, 2021.](https://openaccess.thecvf.com/content/CVPR2021W/AI4Space/papers/Oestreich_On-Orbit_Inspection_of_an_Unknown_Tumbling_Target_Using_NASAs_Astrobee_CVPRW_2021_paper.pdf)
- [K. Albee, C. Specht, H. Mishra, C. Oestreich, B. Brunner, and R. Linares, “Autonomous rendezvous with an uncertain, uncooperative tumbling target: the TumbleDock flight experiments,” presented at the ESA ASTRA, Noordwijk, The Netherlands, Noordwijk, The Netherlands, 2022.](https://elib.dlr.de/189550/)

This repository includes modules for target time-of-flight camera-based state estimation, robust model predictive control, ROS-based coordination code, and sample trajectories. It does not contain DLR's separate motion planning method---a suitable motion planner must be substituted.

This code can also interface with Astrobee's simulation environment for convenient testing. A brief guide has also been produced for [Astrobee's Flight Software](https://github.com/albee/a-brief-guide-to-astrobee) to aid in any Astrobee-related development. 


## Code Structure
TRACE is designed to run on a debian-based system, and has recently been used with both Ubuntu 16.04 and 20.04. Your base system should be
a bare version of either 16.04 or 20.04 (recommended).

*Note: You might also choose to start from NASA Astrobee's preconfigured Ubuntu 20.04 container if you prefer working with Docker. See [below](# Astrobee dependencies).*

TRACE's modules are general-purpose. In this repo, they are integrated with ROS for communication. Full simulation tests (and Astrobee hardware integration) of TRACE are possible using NASA's Astrobee simulator. Discussion of this setup is described [below](#Usage).

## Dependencies and Setup
TRACE's major dependencies include:

- GTSAM
- CasADi
- Teaser++
- ROS (for messaging, Astrobee interfacing, and tf convenience functions)

If you haven't configured ROS yet, add its package lists:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

Most of TRACE's dependencies are satisfied on a debian-based system with:

```bash
# ROS and core dependencies
sudo apt install pip3 python-is-python3 python3-rospkg-modules libgmp3-dev m4 ros-noetic-eigen-conversions libccd-dev libeigen3-dev python-dev python-yaml ros-noetic-desktop-full python3-catkin-tools libpcl-dev wget git

# numerical Python
pip3 install pycddlib scipy numpy pytope matplotlib
```

At this point, you are ready to set up your `trace-ws` workspace for TRACE. If you haven't already sourced ROS, do that now:

```bash
# See ROS install instructions for your system for details if you are not used to this step:
# https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
source /opt/ros/noetic/setup.bash
```

```bash
cd ~
mkdir trace-ws
cd trace-ws
catkin init
git clone https://github.com/albee/trace  # might need to use SSH instead
mv trace src/trace
```

External dependencies (GTSAM, CasADi, Teaser++) are satisfied using the following sources:

```bash
cd src
git submodule update --init --recursive

# gtsam
pushd external/gtsam && mkdir build && cd build && cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON &&  make -j2
sudo make install
popd

# casadi
pushd external/casadi && mkdir build && cd build && cmake .. && make -j2
sudo make install
popd

# teaserpp
pushd external/TEASER-plusplus && mkdir build && cd build && cmake .. -DBUILD_TEASER_FPFH=ON &&  make -j2
sudo make install
popd
```

### Astrobee dependencies

Finally, this implementation of TRACE offers tight integration with the Astrobee simulation for visualization and systems testing. Minimal Astrobee-derived messages and base classes are therefore required; however, TRACE's core algorithms can be separated, if desired:

Follow the instructions from [TUTORIAL.md](TUTORIAL.md/##Astrobee Simulation Setup) for Astrobee simulation setup.

## Build
This repo contains ROS-compatible packages, meant for the `src` directory of a traditional ROS [catkin](http://wiki.ros.org/catkin) workspace.

To build, go to your `trace-ws` workspace :

```
cd ~/trace-ws
catkin build -j2
```

## Usage
See [TUTORIAL.md](TUTORIAL.md) for a walkthrough on running TRACE in the Astrobee simulation environment!


## Methods

### Chaser
- Target state estimation of Target by Chaser (angular velocity and orientation)
- Target parameter estimation of Target by Chaser (in progress)
- Online motion estimation of Target by Chaser
- NLP-based motion planning method which also accounts for appendage avoidance
- Robust tube-MPC controller for robust tracking of trajectory (plus a backup PD controller)

### Target
- Target tracking of desired tumble motion
