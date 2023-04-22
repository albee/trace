/**
 * @file ParamUtils.h
 * @brief Common useful functions for parameter handling.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_PARAMUTILS_H_
#define MIT_SLAM_PARAMUTILS_H_

#include <string>

#include "mit_slam/BlobTracker.h"
#include "mit_slam/CloudOdometer.h"
#include "mit_slam/SlamNode.h"
#include "mit_slam/GraphManager.h"

namespace mit_slam {

/// Builds the parameter structure using values from ROS parameter server to
/// instantiate a `BlobTracker` object. Default values chosen in implementation.
BlobTracker::Params BlobParamsFromRos();

/// Builds the parameter structure using values from ROS parameter server to
/// instantiate a `CloudOdometer` object. Default values chosen in
/// implementation.
CloudOdometer::Params CloudParamsFromRos();

/// Builds the parameter structure using values from ROS parameter server to
/// instantiate a `GraphManager` object. Default values chosen in
/// implementation.
GraphManager::Params GraphParamsFromRos();

}  // namespace slam

#endif  // MIT_SLAM_PARAMUTILS_H_
