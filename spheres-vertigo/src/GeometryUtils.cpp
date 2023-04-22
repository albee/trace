/**
 * @file GeometryUtils.cpp
 * @brief Geometric tools.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include <unsupported/Eigen/MatrixFunctions>

//#include "cpp-toolbox/GeometryUtils.h"
#include "sph-vertigo/GeometryUtils.h"
#include "polhode_alignment/geometry_utils.h"

namespace sph {

/* ************************************************************************** */
Eigen::Vector3d ComputeOmegaParallel(const Eigen::MatrixXd &omegas,
                                     const LLSMethod &lls) {
  Eigen::Vector3d n;
  size_t N = omegas.cols();
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(N);
  Eigen::MatrixXd A = omegas.transpose();

  if (lls == LLSMethod::kSvd) {
    n = omegas.transpose()
            .bdcSvd(Eigen::DecompositionOptions::ComputeThinU |
                    Eigen::DecompositionOptions::ComputeThinV)
            .solve(ones);
  } else if (lls == LLSMethod::kHouseQR) {
    n = omegas.transpose().householderQr().solve(ones);
  } else if (lls == LLSMethod::kColPivHouseQR) {
    n = omegas.transpose().colPivHouseholderQr().solve(ones);
  } else if (lls == LLSMethod::kFullHouseQR) {
    n = omegas.transpose().fullPivHouseholderQr().solve(ones);
  } else {  // (lls == LLSMethod::kNormal)
    n = (omegas * omegas.transpose()).ldlt().solve(omegas * ones);
  }

  return (n / n.squaredNorm());
}

/* ************************************************************************** */
Eigen::MatrixXd RotateIndividualOmegaMatrix(
    const Eigen::MatrixXd &omegas, const std::vector<Eigen::Matrix3d> &R) {
  size_t N = omegas.cols(), NRots = R.size();
  if (N != NRots) {
    std::cout << "(sph::RotateIndividualOmegaMatrix) Mismatched sizes.\n";
    std::exit(1);
  }

  Eigen::MatrixXd rotated = Eigen::MatrixXd::Zero(3, N);
  for (size_t i = 0; i < N; i++) {
    rotated.col(i) = R[i] * omegas.col(i);
  }

  return rotated;
}

/* ************************************************************************** */
Eigen::Vector2d ComputeInertiaRatiosParallel(
    const Eigen::MatrixXd &omegas, const std::vector<Eigen::Matrix3d> &R,
    const LLSMethod &lls) {
  // Rotate the angular velocities to the world frame.
  Eigen::MatrixXd omegaB_W = RotateIndividualOmegaMatrix(omegas, R);
  // Get the parallel component of the angular velocity vector in W frame.
  Eigen::Vector3d omega_parallel = ComputeOmegaParallel(omegaB_W);

  // Build least squares.
  size_t N = omegaB_W.cols();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * N, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * N);
  for (size_t i = 0; i < N; i++) {
    A.block(i * 3, 0, 3, 3) = R[i] * omegas.col(i).asDiagonal();
    b.segment(i * 3, 3) = omega_parallel;
  }

  // Solve least squares.
  Eigen::Vector3d n;
  if (lls == LLSMethod::kSvd) {
    n = A.bdcSvd(Eigen::DecompositionOptions::ComputeThinU |
                 Eigen::DecompositionOptions::ComputeThinV)
            .solve(b);
  } else if (lls == LLSMethod::kHouseQR) {
    n = A.householderQr().solve(b);
  } else if (lls == LLSMethod::kColPivHouseQR) {
    n = A.colPivHouseholderQr().solve(b);
  } else if (lls == LLSMethod::kFullHouseQR) {
    n = A.fullPivHouseholderQr().solve(b);
  } else {  // (lls == LLSMethod::kNormal)
    n = (A.transpose() * A).ldlt().solve(A.transpose() * b);
  }

  Eigen::Vector2d J;
  J(0) = n(0) / n(2);  // J1.
  J(1) = n(1) / n(2);  // J2.

  return J;
}

/* ************************************************************************** */
Eigen::Vector3d Vee(const Eigen::Matrix3d &M) {
  Eigen::Vector3d vec{-M(1, 2), M(0, 2), -M(0, 1)};
  return vec;
}

/* ************************************************************************** */
Eigen::Vector3d Log(const Eigen::Matrix3d &R) {
  if (R.isApprox(Eigen::Matrix3d::Identity())) return Eigen::Vector3d::Zero();
  double mag = std::acos((R.trace() - 1.0) / 2.0);
  Eigen::Vector3d theta = (mag / (2 * std::sin(mag))) * Vee(R - R.transpose());
  return theta;
}

/* ************************************************************************** */
// Eigen::Matrix3d Exp(const Eigen::Vector3d &theta) {
//   Eigen::Matrix3d thetaSkew = Skew(theta);
// }

/* ************************************************************************** */
Eigen::Vector3d ComputeAngularVelocity(const Eigen::Matrix3d &R1,
                                       const Eigen::Matrix3d &R2,
                                       const double deltaT) {
  Eigen::Matrix3d deltaR = R1.transpose() * R2;
  Eigen::Vector3d deltaTheta = Log(deltaR);
  return (1.0 / deltaT) * deltaTheta;
}

// /* ************************************************************************** */
// /*
// void TriangulateMatches(StereoFrame *frame, const StereoCamera &cam,
//                         const bool clip, const double min_depth,
//                         const double max_depth) {
//   std::vector<cv::DMatch> inlier_matches;
//
//   for (auto match : frame->info.matches) {
//     Eigen::Vector3d C_p;  // triangulated point p in camera frame C
//
//     double uL = frame->info.keypts_left[match.queryIdx].pt.x;   // [px]
//     double uR = frame->info.keypts_right[match.trainIdx].pt.x;  // [px]
//     double vL = frame->info.keypts_left[match.queryIdx].pt.y;   // [px]
//     // double v_R = frame.info.keypts_right[match.trainIdx].pt.y; // [px]
//
//     double d = uL - uR;                  // disparity [px]
//     double Tx_over_d = cam.tx / d;       // [m/px]
//     double depth = Tx_over_d * (cam.f);  // [m]
//     if ((d < 0) ||                       // negative disparities are invalid
//         (((depth > max_depth) || (depth < min_depth)) && clip))
//       continue;
//
//     C_p(0) = Tx_over_d * (uL - cam.cx);  // x
//     C_p(1) = Tx_over_d * (vL - cam.cy);  // y
//     C_p(2) = depth;                      // z
//
//     // Find the Jacobian of feature location r = [x y z]' with respect to stereo
//     // image coordinate s = [uL uR vL]'
//     Eigen::Matrix3d Jr_s = Eigen::Matrix3d::Zero();
//     double f = cam.f;
//     double tx = cam.tx;
//     double cx = cam.cx;
//     double cy = cam.cy;
//     Jr_s(0, 0) = tx / (uL - uR) + (tx * (cx - uL)) / pow(uL - uR, 2);  // dx/duL
//     Jr_s(0, 1) = -(tx * (cx - uL)) / pow(uL - uR, 2);                  // dx/duR
//     Jr_s(1, 0) = (tx * (cy - vL)) / pow(uL - uR, 2);                   // dy/duL
//     Jr_s(1, 1) = -(tx * (cy - vL)) / pow(uL - uR, 2);                  // dy/duL
//     Jr_s(1, 2) = tx / (uL - uR);                                       // dy/dvL
//     Jr_s(2, 0) = -(f * tx) / pow(uL - uR, 2);                          // dz/duL
//     Jr_s(2, 1) = (f * tx) / pow(uL - uR, 2);                           // dz/duR
//
//     // Find the covariance of the point r = [x y z]'
//     frame->info.cov_matches3D.push_back(Jr_s * cam.covS * Jr_s.transpose());
//
//     frame->info.matches3D.emplace_back(C_p);
//     inlier_matches.emplace_back(match);
//   }
//   frame->info.matches = inlier_matches;  // replace with physically-consistent
// }
//
// // Reproject a single point back onto the image
// /* ************************************************************************** */
// /*
// Eigen::Vector2d ReprojectToImage(const Eigen::Vector3d &p,
//                                  const StereoCamera &cam) {
//   Eigen::Vector2d pointImage;
//   pointImage(0) = cam.f * p(0) / p(2) + cam.cx;
//   pointImage(1) = cam.f * p(1) / p(2) + cam.cy;
//   return pointImage;
// }

/* ************************************************************************** */
bool PointsTooClose(const std::vector<Eigen::Vector3d> &ptsA,
                    const std::vector<Eigen::Vector3d> &ptsB,
                    const double min_distance) {
  // Assume both sets are the same size.
  for (size_t j = 0; j < ptsA.size(); j++) {
    for (size_t k = j; k < ptsB.size(); k++) {
      // Check all different points to make sure they are not too close.
      if (j != k) {
        Eigen::Vector3d interpointVecA = ptsA[j] - ptsA[k];
        Eigen::Vector3d interpointVecB = ptsB[j] - ptsB[k];
        if (interpointVecA.norm() < min_distance ||
            interpointVecB.norm() < min_distance) {
          return true;
        }
      }
    }
  }
  return false;
}

/* ************************************************************************** */
Eigen::Matrix3d Quat2Rot(const Eigen::Quaterniond &quat) {
  return (quat.conjugate()).toRotationMatrix();
}

// Get the absolute orientation, change in camera pose PAtoB
/* ************************************************************************** */
/*
// void MotionAbsoluteOrientation(const StereoFrame &frameA,
//                                const StereoFrame &frameB,
//                                StereoFrame &inlierFrameA,
//                                StereoFrame &inlierFrameB, Pose3D &deltaPoseAtoB,
//                                const StereoCamera &cam,
//                                const MaxLikelihoodParams &params) {
//   // Find all matches from frameA-to-frameB
//   sph::StereoFrame matchedFrameA = frameA, matchedFrameB = frameB;
//   int numMatchesAtoB = matchedFrameA.info.matches.size();
//   std::cout << "numMatchesAtoB: " << numMatchesAtoB << std::endl;
//   std::cout << "min # inliers: " << params.absoluteOrientationMininmumInliers_
//             << std::endl;
//
//   // Clear the contents of the inlier stereo frames
//   inlierFrameA = frameA;
//   inlierFrameA.info = StereoInfo();
//   inlierFrameB = frameB;
//   inlierFrameB.info = StereoInfo();
//
//   if (numMatchesAtoB < params.absoluteOrientationMininmumInliers_) {
//     // If there are not enough matches, give an invalid result and exit the
//     // function
//     deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
//     deltaPoseAtoB.t << -1, -1, -1;
//     std::cout << "Absolute orientation canceled [matches]: [" << numMatchesAtoB
//               << "]" << std::endl;
//     return;
//   }
//
//   // Store 3D point variables for correspondence
//   std::vector<Eigen::Vector3d> pointsFrameA = matchedFrameA.info.matches3D;
//   std::vector<Eigen::Vector3d> pointsFrameB = matchedFrameB.info.matches3D;
//
//   // Absolute Orientation [Horn] using RANSAC; indices of best inliers and
//   // current inliers
//   std::vector<size_t> inlierIndicesMax, inlierIndicesTemp;
//
//   // Randomly selected pts and whether they are too close together
//   std::vector<Eigen::Vector3d> selectedPointsFrameA, selectedPointsFrameB;
//   bool pointsTooCloseInAllIterations = true;
//
//   // Perform RANSAC iterations
//   int i = 0;  // RANSAC iterator
//   while (i < params.absoluteOrientationRansacMaxIterations_) {
//     // std::cout << "----" << std::endl << "Ransac iter: " << i << std::endl;
//     // Select 4 random points
//     int pointIndex1 = rand() % numMatchesAtoB;
//     int pointIndex2 = rand() % numMatchesAtoB;
//     int pointIndex3 = rand() % numMatchesAtoB;
//     int pointIndex4 = rand() % numMatchesAtoB;
//     // std::cout << "  rdm 4 pts: " << pointIndex1 << ", " << pointIndex2 << ",
//     // "
//     //           << pointIndex3 << ", " << pointIndex4 << std::endl;
//
//     // If indices are not unique, try again
//     if (pointIndex1 == pointIndex2 || pointIndex1 == pointIndex3 ||
//         pointIndex1 == pointIndex4 || pointIndex2 == pointIndex3 ||
//         pointIndex2 == pointIndex4 || pointIndex3 == pointIndex4) {
//       // std::cout << "  xxx indices not unique. aborting cur iter" <<
//       // std::endl;
//       continue;
//     }
//     // Count iterations with unique indices
//     i++;
//
//     // Clear previous points
//     selectedPointsFrameA.clear();
//     selectedPointsFrameB.clear();
//
//     // Select random corresponding points
//     selectedPointsFrameA.push_back(matchedFrameA.info.matches3D[pointIndex1]);
//     selectedPointsFrameA.push_back(matchedFrameA.info.matches3D[pointIndex2]);
//     selectedPointsFrameA.push_back(matchedFrameA.info.matches3D[pointIndex3]);
//     selectedPointsFrameA.push_back(matchedFrameA.info.matches3D[pointIndex4]);
//
//     selectedPointsFrameB.push_back(matchedFrameB.info.matches3D[pointIndex1]);
//     selectedPointsFrameB.push_back(matchedFrameB.info.matches3D[pointIndex2]);
//     selectedPointsFrameB.push_back(matchedFrameB.info.matches3D[pointIndex3]);
//     selectedPointsFrameB.push_back(matchedFrameB.info.matches3D[pointIndex4]);
//
//     // Test if any two points are too close, and continue to the next iteration
//     // if true. This avoids having a similar situation to only having three
//     // points, where 180 deg flips are possible
//     if (PointsTooClose(selectedPointsFrameA, selectedPointsFrameB,
//                        params.aboluteOrientationMinInterpointDist_)) {
//       if (pointsTooCloseInAllIterations &&
//           i == params.absoluteOrientationRansacMaxIterations_) {
//         // If points are too close, try again. If points have been too close in
//         // all iterations, return an invalid result
//         deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
//         deltaPoseAtoB.t << -1, -1, -1;
//         std::cout << "Absolute orientation canceled (points too close in all "
//                      "iterations)"
//                   << std::endl;
//         return;
//       }
//       continue;
//     } else {
//       // If points were not too close in this iteration, note this.
//       pointsTooCloseInAllIterations = false;
//     }
//
//     // Get frame change from A to B
//     MotionAbsoluteOrientationPoints(selectedPointsFrameA, selectedPointsFrameB,
//                                     &deltaPoseAtoB);
//
//     // Determine which of all the matches are inliers (have small reprojection
//     // error) according to the calculated AtoB pose change. Here it is developed
//     // for a static environment, and a moving camera.
//     inlierIndicesTemp.clear();
//     Eigen::Matrix3d RBtoA = deltaPoseAtoB.R;
//     Eigen::Vector3d tAtoB_A = deltaPoseAtoB.t;
//     for (size_t k = 0; k < (size_t)numMatchesAtoB; k++) {
//       // Get reprojection error [px]
//       Eigen::Vector3d rA_A =
//           matchedFrameA.info.matches3D[k];  // pt in frame A in A coords
//       Eigen::Vector3d rB_B =
//           matchedFrameB.info.matches3D[k];  // pt in frame B in B coords
//       Eigen::Vector3d rA_B =
//           RBtoA.transpose() * (rA_A - tAtoB_A);  // pt in frame A in B coords
//
//       Eigen::Vector2d reprojectionError =
//           ReprojectToImage(rB_B, cam) - ReprojectToImage(rA_B, cam);
//       // std::cout << "  reprojectionError: " << reprojectionError.transpose()
//       //           << ", norm: " << reprojectionError.norm() << std::endl;
//
//       // If the reprojection error is smaller than the inlier threshold, add to
//       // inlier indices
//       if (reprojectionError.norm() <
//           params.absoluteOrientationInlierThreshold_) {
//         inlierIndicesTemp.push_back(k);
//       }
//
//       // If the metric error in 3D is smaller than the inlier threshold, add to
//       // inlier indices
//       // if ((rB_B - rA_B).norm() < 0.005) {
//       //   inlierIndicesTemp.push_back(k);
//       // }
//     }
//
//     // If there are more inliers this time than the best run, store temporary
//     // inlier indices
//     if (inlierIndicesTemp.size() > inlierIndicesMax.size()) {
//       inlierIndicesMax = inlierIndicesTemp;
//     }
//
//     // If all features are inliers, stop RANSAC
//     if (static_cast<int>(inlierIndicesMax.size()) == numMatchesAtoB) {
//       break;
//     }
//   }
//
//   // Copy descriptors and keypoints to inlier frames
//   inlierFrameA.info.descr_left = frameA.info.descr_left;
//   inlierFrameA.info.descr_right = frameA.info.descr_right;
//   inlierFrameA.info.keypts_left = frameA.info.keypts_left;
//   inlierFrameA.info.keypts_right = frameA.info.keypts_right;
//
//   inlierFrameB.info.descr_left = frameB.info.descr_left;
//   inlierFrameB.info.descr_right = frameB.info.descr_right;
//   inlierFrameB.info.keypts_left = frameB.info.keypts_left;
//   inlierFrameB.info.keypts_right = frameB.info.keypts_right;
//
//   // Copy inlier matches, their 3D points and their covariances to inlier frames
//   for (size_t l = 0; l < inlierIndicesMax.size(); l++) {
//     inlierFrameA.info.matches.push_back(
//         matchedFrameA.info.matches[inlierIndicesMax[l]]);
//     inlierFrameA.info.matches3D.push_back(
//         matchedFrameA.info.matches3D[inlierIndicesMax[l]]);
//     inlierFrameA.info.cov_matches3D.push_back(
//         matchedFrameA.info.cov_matches3D[inlierIndicesMax[l]]);
//
//     inlierFrameB.info.matches.push_back(
//         matchedFrameB.info.matches[inlierIndicesMax[l]]);
//     inlierFrameB.info.matches3D.push_back(
//         matchedFrameB.info.matches3D[inlierIndicesMax[l]]);
//     inlierFrameB.info.cov_matches3D.push_back(
//         matchedFrameB.info.cov_matches3D[inlierIndicesMax[l]]);
//   }
//
//   // If there are not enough inliers, give an invalid result. Otherwise compute
//   // the least squares solution using the best inliers
//   if (static_cast<int>(inlierFrameA.info.matches.size()) <
//       params.absoluteOrientationMininmumInliers_) {
//     deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
//     deltaPoseAtoB.t << -1, -1, -1;
//     std::cout << "Absolute orientation canceled (only "
//               << inlierFrameA.info.matches.size() << " inliers)" << std::endl;
//
//   } else {
//     MotionAbsoluteOrientationPoints(inlierFrameA.info.matches3D,
//                                     inlierFrameB.info.matches3D,
//                                     &deltaPoseAtoB);
//   }
//
//   std::cout << "AbsOrientation Motion Estimate: " << deltaPoseAtoB << std::endl;
// }
//
// /* ************************************************************************** */
// /*
// void MotionAbsoluteOrientationPoints(
//     const std::vector<Eigen::Vector3d> &pointsFrameA,
//     const std::vector<Eigen::Vector3d> &pointsFrameB, Pose3D *deltaPoseAtoB) {
//   // Find the mean of all the points in each frame
//   Eigen::Vector3d frameAMean = Eigen::Vector3d::Zero();
//   Eigen::Vector3d frameBMean = Eigen::Vector3d::Zero();
//   for (size_t i = 0; i < pointsFrameA.size(); i++) {
//     frameAMean += pointsFrameA[i];
//     frameBMean += pointsFrameB[i];
//   }
//   frameAMean /= static_cast<double>(pointsFrameA.size());
//   frameBMean /= static_cast<double>(pointsFrameA.size());
//
//   // Create M matrix for optimization [Horn]
//   Eigen::Matrix3d M = Eigen::Matrix3d::Zero();  // matrix M from Horn
//   for (size_t j = 0; j < pointsFrameA.size(); j++) {
//     Eigen::Vector3d rA_Ap =
//         pointsFrameA[j] - frameAMean;  // shifted pt in frame A in A coords
//     Eigen::Vector3d rB_Bp =
//         pointsFrameB[j] - frameBMean;  // shifted pt in frame B in B coords
//     Eigen::Matrix3d dM;                // change in matrix M
//
//     dM << rA_Ap(0) * rB_Bp(0), rA_Ap(0) * rB_Bp(1), rA_Ap(0) * rB_Bp(2),
//         rA_Ap(1) * rB_Bp(0), rA_Ap(1) * rB_Bp(1), rA_Ap(1) * rB_Bp(2),
//         rA_Ap(2) * rB_Bp(0), rA_Ap(2) * rB_Bp(1), rA_Ap(2) * rB_Bp(2);
//     M += dM;  // add dM to matrix M
//   }
//
//   // Calculate N = sum_{j} ( qbarmat([rA_Ap_j; 0])' * qmat([rB_Bp_j; 0]') )
//   // From Horn, but adapted for q = [qv qs]'
//   Eigen::Matrix4d N = Eigen::Matrix4d::Zero();  // create N matrix
//   N << M(0, 0) - M(1, 1) - M(2, 2), M(0, 1) + M(1, 0), M(2, 0) + M(0, 2),
//       M(1, 2) - M(2, 1), M(0, 1) + M(1, 0), -M(0, 0) + M(1, 1) - M(2, 2),
//       M(1, 2) + M(2, 1), M(2, 0) - M(0, 2), M(2, 0) + M(0, 2),
//       M(1, 2) + M(2, 1), -M(0, 0) - M(1, 1) + M(2, 2), M(0, 1) - M(1, 0),
//       M(1, 2) - M(2, 1), M(2, 0) - M(0, 2), M(0, 1) - M(1, 0),
//       M(0, 0) + M(1, 1) + M(2, 2);
//
//   // Create eigenvalues / eigenvectors solver and get the real part of the
//   // eigenvalues http://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
//   Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(N, true);
//   Eigen::Vector4cd eigenValuesComplex = eigenSolver.eigenvalues();
//   Eigen::Vector4d eigenValues = eigenValuesComplex.real();
//
//   // Get the index of the maximum eigenvalue and associated eigenvector, which
//   // corresponds to q
//   size_t maxEigenValueCol =
//       (eigenValues(0) > eigenValues(1) && eigenValues(0) > eigenValues(2) &&
//        eigenValues(0) > eigenValues(3))
//           ? 0
//           : ((eigenValues(1) > eigenValues(2) &&
//               eigenValues(1) > eigenValues(3))
//                  ? 1
//                  : ((eigenValues(2) > eigenValues(3)) ? 2 : 3));
//   Eigen::Vector4cd qBtoAComplex =
//       eigenSolver.eigenvectors().col(maxEigenValueCol);
//   Eigen::Vector4d qBtoA = qBtoAComplex.real();
//   qBtoA /= qBtoA.norm();
//
//   // Note that in [Horn1987] his convention leads to the point rotation rather
//   // than the frame change rotation. Return the rotation matrix RBtoA and
//   // translation matrix tAtoB_A
//   Eigen::Quaterniond qRet(qBtoA(3), qBtoA(0), qBtoA(1), qBtoA(2));
//   deltaPoseAtoB->R = Quat2Rot(qRet);
//   deltaPoseAtoB->t = frameAMean - deltaPoseAtoB->R * frameBMean;
// }
//
// /* ************************************************************************** */
// /*
// void RansacAbsoluteOrientation(const std::vector<Eigen::Vector3d> &ptsA,
//                                const std::vector<Eigen::Vector3d> &ptsB,
//                                std::vector<int> *inliers, Pose3D *deltaPoseAtoB,
//                                const StereoCamera &cam, const int num_iters,
//                                const double inlier_tol,
//                                const double close_distance) {
//   int num_meas = ptsA.size();
//   if (num_meas <= 4) {
//     std::cout << "(RansacAbsoluteOrientation) Not enough measurements: "
//               << num_meas << std::endl;
//     return;
//   }
//   // Setup random sampler to pick out our random 4 points.
//   std::default_random_engine eng;
//   std::uniform_int_distribution<int> dist(0, num_meas - 1);
//
//   std::vector<int> best_inlier_set;  // Empty at first.
//   double best_error_yet = 1e10;      // The smaller the better.
//
//   // For each RANSAC iteration.
//   for (int i = 0; i < num_iters; i++) {
//     // Random sampling until we get 4 different points.
//     int idx1 = 0, idx2 = 0, idx3 = 0, idx4 = 0;
//     while (idx1 == idx2 || idx1 == idx3 || idx1 == idx4 || idx2 == idx3 ||
//            idx2 == idx4 || idx3 == idx4) {
//       idx1 = dist(eng);
//       idx2 = dist(eng);
//       idx3 = dist(eng);
//       idx4 = dist(eng);
//     }
//
//     std::vector<Eigen::Vector3d> ptsSubA{ptsA[idx1], ptsA[idx2], ptsA[idx3],
//                                          ptsA[idx4]};
//     std::vector<Eigen::Vector3d> ptsSubB{ptsB[idx1], ptsB[idx2], ptsB[idx3],
//                                          ptsB[idx4]};
//     // - Check if they're too close.
//     if (!PointsTooClose(ptsSubA, ptsSubB, close_distance)) {
//       Pose3D deltaPose;
//       // - Get absolute orientation.
//       MotionAbsoluteOrientationPoints(ptsSubA, ptsSubB, &deltaPose);
//       Eigen::Matrix3d R_AB = deltaPose.R;
//       Eigen::Vector3d t_BwrtA_A = deltaPose.t;
//
//       std::vector<int> cur_inlier_set;
//       double cur_error = 0;
//
//       // - Check for inliers.
//       for (int j = 0; j < num_meas; j++) {
//         // std::cout << "--- pt" << j << std::endl;
//         Eigen::Vector3d t_PwrtA_A = ptsA[j];
//         Eigen::Vector3d t_PwrtB_B = ptsB[j];
//         Eigen::Vector3d tHat_PwrtB_B =
//             R_AB.transpose() * (t_PwrtA_A - t_BwrtA_A);
//
//         Eigen::Vector3d errorVec3D = t_PwrtB_B - tHat_PwrtB_B;
//         double error3D = errorVec3D.norm();
//         Eigen::Vector2d errorVecReprojection =
//             ReprojectToImage(t_PwrtB_B, cam) -
//             ReprojectToImage(tHat_PwrtB_B, cam);
//         double errorReprojection = errorVecReprojection.norm();
//         // std::cout << "Point in B: " << t_PwrtB_B.transpose() << std::endl;
//         // std::cout << "Estim of B: " << tHat_PwrtB_B.transpose() << std::endl;
//         // std::cout << "3D error V: " << errorVec3D.transpose() << std::endl;
//         // std::cout << "3D error  : " << error3D << std::endl;
//         // std::cout << "Repr actl : "
//         //           << ReprojectToImage(t_PwrtB_B, cam).transpose() <<
//         //           std::endl;
//         // std::cout << "Repr xhat : "
//         //           << ReprojectToImage(tHat_PwrtB_B, cam).transpose()
//         //           << std::endl;
//         // std::cout << "Repr error: " << errorReprojection << std::endl;
//
//         if (errorReprojection < inlier_tol) {
//           // std::cout << "  +++ Found an inlier!" << std::endl;
//           cur_inlier_set.push_back(j);
//           cur_error += errorReprojection;
//         }
//       }
//
//       // - Score current solution.
//       if (cur_inlier_set.size() == num_meas) {
//         // If all features are inliers, stop RANSAC.
//         std::cout << " +!+!+! Congrats. All inliers!" << std::endl;
//         best_inlier_set = cur_inlier_set;
//         best_error_yet = cur_error;
//         break;
//       }
//
//       // If we have more inliers than before, keep. If same, compare scores.
//       if (cur_inlier_set.size() > best_inlier_set.size()) {
//         best_inlier_set = cur_inlier_set;
//         best_error_yet = cur_error;
//       } else if (cur_inlier_set.size() == best_inlier_set.size()) {
//         if (cur_error < best_error_yet) {
//           best_inlier_set = cur_inlier_set;
//           best_error_yet = cur_error;
//         }
//       }
//
//     } else {
//       // Check to see if it's the last iteration and return error msg.
//     }
//   }
//
//   // Solve once more with all the inliers.
//   std::vector<Eigen::Vector3d> inlierPtsA, inlierPtsB;
//   for (const int in : best_inlier_set) {
//     inlierPtsA.push_back(ptsA[in]);
//     inlierPtsB.push_back(ptsB[in]);
//   }
//   // Return best solution.
//   if (inlierPtsA.size() < 4) {
//     std::cout << "(RansacAbsoluteOrientation) Less than 4 inliers: "
//               << inlierPtsA.size() << std::endl;
//   } else {
//     std::cout << "(RansacAbsoluteOrientation) Success with "
//               << inlierPtsA.size() << std::endl;
//     MotionAbsoluteOrientationPoints(inlierPtsA, inlierPtsB, deltaPoseAtoB);
//   }
//   *inliers = best_inlier_set;
// }
//
// /* ************************************************************************** */
// // Get the maximum likelihood estimate for pose change of the camera from A to B
// // given corresponding points and their covariance [Adapted from Matthies
// // PhD 2.3.2 Appendix B.2, see Setterfield Thesis Proposal]. Return the inlier
// // points, the pose change, and the covariance of the estimate.
// /*
// void MotionMaximumLikelihood(const StereoFrame &frameA,
//                              const StereoFrame &frameB,
//                              StereoFrame &inlierFrameA,
//                              StereoFrame &inlierFrameB, Pose3D &deltaPoseAtoB,
//                              const StereoCamera &cam,
//                              const MaxLikelihoodParams &params) {
//   // Get the absolute orientation and inlier frames from frame A to frame B.
//   // This will eliminate features that do not pass the rigidity check, and give
//   // an initialization pose for the maximum likelihood estimate.
//   MotionAbsoluteOrientation(frameA, frameB, inlierFrameA, inlierFrameB,
//                             deltaPoseAtoB, cam, params);
//
//   // If no valid absolute orientation was found, return the invalid results
//   if (deltaPoseAtoB.R.determinant() == -1) {
//     deltaPoseAtoB.cov = Eigen::Matrix<double, 6, 6>::Zero();
//     return;
//   }
//
//   // Initialize using absolute orientation results
//   Eigen::Matrix3d RAtoB = deltaPoseAtoB.R.transpose();  // rot from A to B
//   Eigen::Vector3d tAtoB_B =
//       deltaPoseAtoB.R * deltaPoseAtoB.t;  // trans from A to B in B coords
//
//   // Initialize delTheta, the so(3) change in rotation solved for at each
//   // iteration
//   Eigen::Vector3d delTheta;
//   delTheta << -1, -1, -1;
//
//   // Initialize sum terms required for solving delTheta = (A - B*inv(C)*B')\(D -
//   // B*inv(C)*E)
//   Eigen::Matrix3d A, B, C, C_inv;
//   Eigen::Vector3d D, E;
//
//   // Initialize sum term required for solving cov([delTheta delT]') = inv(F)
//   Eigen::Matrix<double, 6, 6> F;
//
//   // Solve the linearized problem until the orientation change is small
//   int iterations = 0;
//   while (delTheta.norm() > params.maxLikelihoodDelThetaThreshold_ &&
//          iterations < params.maxLikelihoodMaxIterations_) {
//     // Reset sum terms to zero
//     A = Eigen::Matrix3d::Zero();
//     B = Eigen::Matrix3d::Zero();
//     C = Eigen::Matrix3d::Zero();
//     D = Eigen::Vector3d::Zero();
//     E = Eigen::Vector3d::Zero();
//     F = Eigen::Matrix<double, 6, 6>::Zero();
//
//     // Loop through all features in the inlier frames and get the necessary
//     // matrices
//     for (int j = 0; j < static_cast<int>(inlierFrameA.info.matches.size());
//          j++) {
//       Eigen::Vector3d rA_A, rB_B, Q_j;
//       Eigen::Matrix3d rA_Ax, R_j, R_j_inv, J_j;
//       Eigen::Matrix<double, 3, 6> H_j;
//
//       rA_A = inlierFrameA.info.matches3D[j];  // points in frame A in A coords
//       rB_B = inlierFrameB.info.matches3D[j];  // points in frame B in B coords
//
//       // Covariance of point position error (inverse of this used for weighting)
//       R_j = inlierFrameB.info.cov_matches3D[j] +
//             RAtoB * inlierFrameA.info.cov_matches3D[j] * RAtoB.transpose();
//
//       Q_j = rB_B - RAtoB * rA_A + tAtoB_B;  // Q_j term
//       rA_Ax = pao::SkewMat(rA_A);              // skew symmetric matrix from rA_A
//       J_j = -RAtoB * rA_Ax;  // lin. sensitivity of predicted rB_B to delTheta
//       H_j.block(0, 0, 3, 3) = J_j;  // H_j term
//       H_j.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
//
//       R_j_inv = R_j.inverse();  // inverse of covariance matrix
//
//       // Calculate the sum terms
//       A = A + J_j.transpose() * R_j_inv * J_j;
//       B = B + J_j.transpose() * R_j_inv;
//       C = C + R_j_inv;
//       D = D + J_j.transpose() * R_j_inv * Q_j;
//       E = E + R_j_inv * Q_j;
//       F = F + H_j.transpose() * R_j_inv * H_j;
//     }
//
//     // Solve for the so(3) change in linearized rotation
//     C_inv = C.inverse();
//     delTheta = (A - B * C_inv * B.transpose()).inverse() * (D - B * C_inv * E);
//
//     // Update the rotation matrix
//     RAtoB = RAtoB * (Eigen::Matrix3d(pao::SkewMat(delTheta))).exp();
//
//     // Change in translation
//     // TODO(tim): can expmap of delTheta be incorporated into this update?
//     Eigen::Vector3d delT = C_inv * (E - B.transpose() * delTheta);
//     tAtoB_B = tAtoB_B - delT;
//
//     // Increment iteration count
//     iterations++;
//   }
//
//   // Form the pose PBtoA = [RAtoB tBtoA_B; 0 0 0 1]
//   sph::Pose3D PBtoA(RAtoB, -tAtoB_B);
//
//   // Store the rotation from B to A
//   Eigen::Matrix3d RBtoA = RAtoB.transpose();
//
//   // updated - need to test
//   // Get the found covariance, that is the covariance of RAtoB and tAtoB_B and
//   // make it valid for PBtoA (see logbook #3 pp 149)
//   Eigen::Matrix<double, 6, 6> covFound = F.inverse();
//   PBtoA.cov.block(0, 0, 3, 3) = covFound.block(0, 0, 3, 3);
//   PBtoA.cov.block(3, 3, 3, 3) =
//       RBtoA * covFound.block(3, 3, 3, 3) * RBtoA.transpose();
//   PBtoA.cov.block(0, 3, 3, 3) = covFound.block(0, 3, 3, 3) * RBtoA.transpose();
//   PBtoA.cov.block(3, 0, 3, 3) = PBtoA.cov.block(0, 3, 3, 3).transpose();
//
//   // Invert the pose to find PAtoB = [RBtoA tAtoB_A; 0 0 0 1] with proper
//   // covariance
//   deltaPoseAtoB = PBtoA.inverse();
//
//   /* old - known to work
//   deltaPoseAtoB.R 		= RBtoA;
//
//   // Translation from A to B in A coordinates and covariance (expected value of
//   // [delTheta RBtoA*delT] [delTheta RBtoA*delT]'). Note that the uncertainty in
//   rotation is
//   // not altered during the inversion, but the uncertainty in translation is.
//   deltaPoseAtoB.t   				 = RBtoA * tAtoB_B;
//   deltaPoseAtoB.cov 				 = F.inverse();
//   deltaPoseAtoB.cov.block(3,3,3,3) = RBtoA * deltaPoseAtoB.cov.block(3,3,3,3) *
//   RBtoA.transpose();
//   */
//
//   std::cout << "MaxLikelihood Motion Estimate: " << deltaPoseAtoB << std::endl;
//  }

}  // namespace sph
