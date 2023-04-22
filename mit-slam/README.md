# SLAM Nodelet

Nodelet for simultaneous localization and mapping (SLAM) tasks to estimate the state of both the chaser and target, as well as the target's inertia tensor.

The chaser's HazCam produces point clouds of the tumbling target. Features are detected in each frame's point cloud, which are matched and registered across frames to provide pose odometry measurements of the target. The chaser's IMU provides pose odometry measurements of the chaser. 

These measurements are both utilized in a factor graph that links the poses of both the chaser and target in the world frame. Via incremental smoothing and mapping (iSAM), the factor graph is solved at each frame, thus providing up-to-date state measurements of the chaser and target. 

The target inertia is estimated once enough target state data is accumulated (TODO: do this via Tim Setterfield's method).

## Building with Astrobee

The SLAM nodelet depends on the Point Cloud Library (PCL) for point cloud detection/matching, Teaser++ for point cloud registration, and GTSAM for factor graphs/iSAM.

### PCL

(For Astrobee use: this is already compiled as part of Astrobee's software, so there is nothing needed to be done here.)

### Teaser++

This automatically compiled as part of the TRACE code, so there aren't any extra steps.

### GTSAM

This automatically compiled as part of the TRACE code, so there aren't any extra steps.

This must be installed before compiling the TRACE. Using the forked repo at https://github.com/coestreich/gtsam (make sure to use the `develop` branch!), you can follow these steps to properly install it on your system:

```bash
git clone -b develop https://github.com/coestreich/gtsam
cd gtsam && mkdir build && cd build
cmake ..
make
sudo make install
```

## Running online with Astrobee

The SLAM nodelet is set to automatically launch with other TD nodelets. The estimation will begin right away, providing state estimates of the chaser and target at 1 Hz (or whatever the HazCam rate is set at). A full HazCam callback function to produce the updated state estimate currently takes about 0.1 - 0.2 seconds.

The SLAM design parameters are set via the files in the `config` folder. The `blob` params refer to tracking the target's centroid, the `cloud` parameters refer to the point cloud odometry and Teaser++ solver, and the `graph` parameters refer to the GTSAM factor graph solving process. The `slam` params provide high-level parameters to run the whole process. These `.yaml` files are called in `MLP.launch` before the nodelet is launched.

## Running with pre-recorded bag data.

For ease of development, the SLAM nodelet can be run on pre-recorded Astrobee simulator data. This also serves as a work-around for the chattering IMU issue, as this data was collected using the teleop_tool for smooth chaser movement. The bag file can be downloaded here: https://drive.google.com/file/d/1XyB_OAnV6vqDHEJJCWSZoTWVa71NpdQX/view?usp=sharing. As a standalone nodelet, the full HazCam callback function currently takes about 0.05 - 0.1 seconds. Here are the steps to run the SLAM nodelet and visualize the point clouds in RViz:

1. Terminal #1: `roscore`
2. Terminal #2: `rosparam set /use_sim_time true`
3. Terminal #2: `rviz -d point_cloud_view.rviz` (from the `/develop/rviz`) folder.
4. Terminal #3: `roslaunch mit_slam mit_slam.launch`
5. Terminal #4: `rosbag play --clock <NAME_OF_BAG_FILE>.bag`

## TODOs

1. Set up and validate a unit test with a moving chaser (requires fixing the chattering IMU issue).
2. Add inertia estimates via batch processing (see Tim Setterfield's method).
3. Debug loop closure errors with iSAM. Note that this isn't strictly required for SLAM, but it would likely help the state estimate accuracy.
4. Lots of tuning based on dry-run testing.

## SLAM pipeline overview

### Point Cloud Callback

1. Obtain HazCam point cloud of target.
2. Remove background points, downsample if necessary to improve speed.
3. Detect FPFH features using PCL/Teaser++.
4. Match FPFH features to previous frame's point cloud using PCL/Teaser++.
5. Register matches to provide a pose odometry measurement  using Teaser++.
6. Add pose odometry measurement to iSAM factor graph using GTSAM.
7. Add chaser IMU integrated measurement to iSAM factor graph using GTSAM.
8. Add range/bearing measurement of target point cloud to iSAM factor graph using GTSAM.
9. Add rotation kinematic factor to iSAM factor graph using GTSAM.
10. Solve for chaser and target pose in world frame using GTSAM.
11. Estimate angular velocity of chaser and target using the pose estimates.
12. Publish front-end measurements and state estimates.

### IMU Callback

1. Integrate chaser IMU acceleration and angular velocity using GTSAM.

### Inertia Estimation (TODO)

1. After a defined number of estimates are obtained, estimate the inertia tensor that best characterizes the observed target dynamics.

