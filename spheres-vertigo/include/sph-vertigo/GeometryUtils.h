/**
 * @file GeometryUtils.h
 * @brief Geometric tools.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_GEOMETRYUTILS_H_
#define SPH_VERTIGO_GEOMETRYUTILS_H_

#include <string>
#include <vector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/// Supported methods for solving a linear least squares problem.
enum class LLSMethod {
  kSvd,            ///< Slowest by most accurate, divide and conquer SVD.
  kHouseQR,        ///< Fast but unstable, Householder QR.
  kColPivHouseQR,  ///< Bit slower but more accurate, column pivoting.
  kFullHouseQR,    ///< Slow but stable, full pivoting.
  kNormal          ///< Fastest but least accurate, normal equations.
};

/**
 * @brief Compute LS estimate of omega component parallel to inertial ang. mom.
 * @param[in] omegas  3xN matrix of body angular velocities in inertial frame W.
 * @param[in] lls     Desired method for solving the linear least squares.
 * @return The omega component parallel to the inertial angular momentum vector.
 *
 * Solves the linear least squares problem for n = omega_par/||omega_par||^2:
 *
 *  n^* = argmin_{n} || omegas^T * n - 1 ||^2,
 *
 * returning (n^* / ||n^*||^2) = omega_par^* as explained in [SetterfieldPhD,
 * pp.119-120] and [Masutani...].
 */
Eigen::Vector3d ComputeOmegaParallel(const Eigen::MatrixXd &omegas,
                                     const LLSMethod &lls = LLSMethod::kSvd);

/**
 * @brief Rotate individual angular velocity vectors by corresponding R matrix.
 * @param[in] omegas  3xN matrix of body angular velocities.
 * @param[in] R       N-size vector with SO(3) rotation matrices.
 * @return The rotated angular velocities' matrix.
 */
Eigen::MatrixXd RotateIndividualOmegaMatrix(
    const Eigen::MatrixXd &omegas, const std::vector<Eigen::Matrix3d> &R);

/**
 * @brief Compute inertia ratios estimate using parallel omega component.
 * @param[in] omegas  3xN matrix of body angular velocities in body frame B.
 * @param[in] R       N-size vector with SO(3) rotation matrices R_WB.
 * @param[in] lls     Desired method for solving the linear least squares.
 *
 * Solves the linear least squares problem for n = (2E/h^2) [Ix,Iy,Iz]^T:
 *
 *  n^* = argmin_{n} || [R_WBi diag(wi)] n - omega_parallel ||^2,
 *
 * then normalizing by the last component to get the inertia ratios J1,J2, as
 * described in [SetterfieldPhD, pp.120-121].
 */
Eigen::Vector2d ComputeInertiaRatiosParallel(
    const Eigen::MatrixXd &omegas, const std::vector<Eigen::Matrix3d> &R,
    const LLSMethod &lls = LLSMethod::kSvd);

/**
 * @brief Get the approximate angular velocity between two attitude values.
 * @param R1      Rotation matrix in SO(3) at time 1.
 * @param R2      Rotation matrix in SO(3) at time 2.
 * @param deltaT  Time difference between time 2 and time 1.
 * @return Resulting angular velocity vector from the difference in attitude.
 */
Eigen::Vector3d ComputeAngularVelocity(const Eigen::Matrix3d &R1,
                                       const Eigen::Matrix3d &R2,
                                       const double deltaT);

/**
 * @brief Get the vector corresponding to a skew symmetric cross product matrix.
 * @param M  Skew symmetric cross product matrix.
 * @return Vector corresponding to the skew symmetric cross product matrix.
 */
Eigen::Vector3d Vee(const Eigen::Matrix3d &M);

/**
 * @brief Compute logarithmic map of SO(3) and vee operator to get theta vec.
 * @param R  Rotation matrix in SO(3).
 * @return Angle-axis representation of the rotation matrix.
 */
Eigen::Vector3d Log(const Eigen::Matrix3d &R);

template <typename T>
T sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief Compute the exponential map of SO(3) to get rotation matrix from vec.
 * @param theta  The angle-axis representation of the rotation matrix.
 * @return Rotation matrix in SO(3).
 */
// Eigen::Matrix3d Exp(const Eigen::Vector3d &theta);

/**
 * @brief 2D-3D stereo triangulation for all tracked features in frame.
 * @param[in/out] frame Processed VERTIGO frame with matches and keypoints.
 * @param[in] cam Stereo camera object that holds calibration parameters.
 * @param[in] clip Whether to discard features further away than `max_depth`.
 * @param[in] max_depth Specifies the clipping distance, in meters.
 *
 * Discards outlier correspondences (negative disparities), replacing the
 * `info.matches` with the ones corresponding to the inlier correspondences from
 * `info.matches3D`, i.e., the triangulated features with positive disparities.
 */

// void TriangulateMatches(StereoFrame *frame, const StereoCamera &cam,
//                         const bool clip = true, const double min_depth = 0.1,
//                         const double max_depth = 1.5);
//
// /**
//  * @brief Reproject a single point back onto the image.
//  */
// Eigen::Vector2d ReprojectToImage(const Eigen::Vector3d &p,
//                                  const StereoCamera &cam);

/**
 * Test if any two points are too close, and continue to the next iteration if
 * true. This avoids having a similar situation to only having three points,
 * where 180 deg flips are possible.
 */
bool PointsTooClose(const std::vector<Eigen::Vector3d> &ptsA,
                    const std::vector<Eigen::Vector3d> &ptsB,
                    const double min_distance);

/**
 * @brief Return the rotation matrix corresponding to a quaternion.
 *
 * Convert from quaternion to rotation matrix, using SPHERES conventions.
 */
Eigen::Matrix3d Quat2Rot(const Eigen::Quaterniond &quat);

/**
 */
struct MaxLikelihoodParams {
  // threshold for how close a feature's reprojection needs to be to be
  // considered an inlier [px]
  double absoluteOrientationInlierThreshold_ =
      2.5;  // reprojection inlier tolerance [px]

  // maximum number of ransac iterations for absolute orientation
  double absoluteOrientationRansacMaxIterations_ =
      5000;  // max iterations for RANSAC [-]

  // minimum number of inliers in absolute orientation
  double absoluteOrientationMininmumInliers_ = 6;  // min number of inliers [-]

  // minimum distance between any two ransac points selected for absolute
  // orientation [m]
  double aboluteOrientationMinInterpointDist_ =
      0.005;  // min dist between any two RANSAC points [m]

  // threshold for norm(deltheta) for which to consider converged [-]
  double maxLikelihoodDelThetaThreshold_ =
      6e-6;  // convergence threshold for norm(delTheta) [-]

  double maxLikelihoodMaxIterations_ =
      25;  // max iterations for max likelihood [-]
};

/**
 */
// void MotionAbsoluteOrientation(
//     const StereoFrame &frameA, const StereoFrame &frameB,
//     StereoFrame &inlierFrameA, StereoFrame &inlierFrameB, Pose3D &deltaPoseAtoB,
//     const StereoCamera &cam,
//     const MaxLikelihoodParams &params = MaxLikelihoodParams());
//
// /**
//  * @brief
//  * @param[in] frameA
//  * @param[in] frameB
//  * @param[out] inlierFrameA
//  * @param[out] inlierFrameB
//  * @param[out] deltaPoseAtoB
//  * @param[in] cam
//  * @param[in] params
//  */
// void MotionMaximumLikelihood(
//     const StereoFrame &frameA, const StereoFrame &frameB,
//     StereoFrame &inlierFrameA, StereoFrame &inlierFrameB, Pose3D &deltaPoseAtoB,
//     const StereoCamera &cam,
//     const MaxLikelihoodParams &params = MaxLikelihoodParams());
//
// /**
//  * @brief Compute Horn's absolute orientation and recover translation.
//  * @param[in] pointsFrameA Set of 3D points expressed in A.
//  * @param[in] pointsFrameB Same set as pointsFrameA, but expressed wrt B.
//  * @param[out] deltaPoseAtoB Computed SE(3) transformation from A to B.
//  *
//  * Get the pose change of the camera from frames A to B (i.e. RBtoA, and
//  * tAtoB_A) given corresponding 3D points using the closed form solution to
//  * absolute orientation [Horn1987]. Here it is developed for a static
//  * environment and a moving camera.
//  */
// void MotionAbsoluteOrientationPoints(
//     const std::vector<Eigen::Vector3d> &pointsFrameA,
//     const std::vector<Eigen::Vector3d> &pointsFrameB, Pose3D *deltaPoseAtoB);
//
// /**
//  * @brief RANSAC version of MotionAbsoluteOrientationPoints.
//  * @param[in] ptsA Set of 3D points expressed in A.
//  * @param[in] ptsB Same set as ptsA, but expressed wrt B.
//  * @param[out] inliers Vector of the indices marked as inliers.
//  * @param[out] deltaPoseAtoB Computed SE(3) transformation from A to B.
//  * @param[in] cam Stereo camera used for reprojection error.
//  * @param[in] inlier_tol Threshold for feature's reprojection 2B inlier [px].
//  * @param[in] num_iters Max number of RANSAC iterations 4 absolute orientation.
//  * @param[in] close_distance Min distance between any two RANSAC points [m].
//  */
// void RansacAbsoluteOrientation(const std::vector<Eigen::Vector3d> &ptsA,
//                                const std::vector<Eigen::Vector3d> &ptsB,
//                                std::vector<int> *inliers, Pose3D *deltaPoseAtoB,
//                                const StereoCamera &cam,
//                                const int num_iters = 500,
//                                const double inlier_tol = 2.5,
//                                const double close_distance = 0.005);

}  // namespace sph

#endif  // SPH_VERTIGO_GEOMETRYUTILS_H_
