/**
 * @file VisualOdometer.h
 * @brief Visual odometry through absolute orientation, max motion likelihood.
 * @date August 01, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 * Adapted from Tim Setterfield's VERTIGO libraries.
 */

#ifndef VISUALODOMETER_H_
#define VISUALODOMETER_H_

#include <opencv2/core/core.hpp>

#include "sph-vertigo/CommonTypes.h"
#include "sph-vertigo/FeatureHandler.h"

namespace sph {

class VisualOdometer {
  // Threshold for how close a feature's reprojection needs to be to be
  // considered an inlier [px]
  double absoluteOrientationInlierThreshold_;

  // Maximum number of RANSAC iterations for absolute orientation
  int absoluteOrientationRansacMaxIterations_;

  // Minimum number of inliers in absolute orientation
  int absoluteOrientationMininmumInliers_;

  // Minimum distance between any two RANSAC points selected for absolute
  // orientation [m]
  double aboluteOrientationMinInterpointDist_;

  // Threshold for norm(delTheta) for which to consider converged [-]
  double maxLikelihoodDelThetaThreshold_;

  double maxLikelihoodMaxIterations_;

  bool verbose_ = false;

  // Camera structure if stereo camera being used
  StereoCamera* stereoCamera_;

  // A feature handler used for feature detection, extraction, and matching
  FeatureHandler* featureHandler_;

 public:
  VisualOdometer(StereoCamera* stereoCamera, FeatureHandler* featureHandler);
  virtual ~VisualOdometer();
  void calculateMotionAbsoluteOrientation(const omsci::StereoFrame& frameA,
                                          const omsci::StereoFrame& frameB,
                                          omsci::StereoFrame& inlierFrameA,
                                          omsci::StereoFrame& inlierFrameB,
                                          Pose3D& deltaPoseAtoB);
  void calculateMotionMaximumLikelihood(const omsci::StereoFrame& frameA,
                                        const omsci::StereoFrame& frameB,
                                        omsci::StereoFrame& inlierFrameA,
                                        omsci::StereoFrame& inlierFrameB,
                                        Pose3D& deltaPoseAtoB);

 private:
  void calculateMotionAbsoluteOrientationPoints(
      const std::vector<Eigen::Vector3d>& pointsFrameA,
      const std::vector<Eigen::Vector3d>& pointsFrameB, Pose3D& deltaPoseAtoB);
};

}  // namespace sph

#endif /* VISUALODOMETER_H_ */
