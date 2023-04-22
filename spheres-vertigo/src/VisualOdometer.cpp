#include <unsupported/Eigen/MatrixFunctions>

#include "sph-vertigo/VisualOdometer.h"

namespace sph {

// Constructor
VisualOdometer::VisualOdometer(StereoCamera* stereoCamera,
                               FeatureHandler* featureHandler)
    : stereoCamera_(stereoCamera), featureHandler_(featureHandler) {
  absoluteOrientationInlierThreshold_ =
      2.5;  // reprojection inlier tolerance [px]
  absoluteOrientationRansacMaxIterations_ =
      250;  // max iterations for RANSAC [-]
  // 250;                                  // max iterations for RANSAC [-]
  absoluteOrientationMininmumInliers_ = 6;  // min number of inliers [-]
  aboluteOrientationMinInterpointDist_ =
      0.005;  // min dist between any two RANSAC points [m]
  maxLikelihoodDelThetaThreshold_ =
      6e-6;  // convergence threshold for norm(delTheta) [-]
  maxLikelihoodMaxIterations_ = 25;  // max iterations for max likelihood [-]
}

VisualOdometer::~VisualOdometer() {}

// Get the absolute orientation, change in camera pose PAtoB
void VisualOdometer::calculateMotionAbsoluteOrientation(
    const omsci::StereoFrame& frameA, const omsci::StereoFrame& frameB,
    omsci::StereoFrame& inlierFrameA, omsci::StereoFrame& inlierFrameB,
    Pose3D& deltaPoseAtoB) {
  // Find all matches from frameA-to-frameB
  omsci::StereoFrame matchedFrameA, matchedFrameB;
  featureHandler_->matchStereoFrames(frameA, frameB, matchedFrameA,
                                     matchedFrameB);
  int numMatchesAtoB = matchedFrameA.leftRightMatches.size();

  // Clear the contents of the inlier stereo frames
  inlierFrameA = omsci::StereoFrame();
  inlierFrameB = omsci::StereoFrame();

  if (numMatchesAtoB < absoluteOrientationMininmumInliers_) {
    // If there are not enough matches, give an invalid result and exit the
    // function
    deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
    deltaPoseAtoB.t << -1, -1, -1;
    if (verbose_) {
      std::cout << "Absolute orientation canceled [matches]: ["
                << numMatchesAtoB << "]" << std::endl;
    }
    return;
  }

  // Store 3D point variables for correspondence
  std::vector<Eigen::Vector3d> pointsFrameA = matchedFrameA.leftRightMatches3D;
  std::vector<Eigen::Vector3d> pointsFrameB = matchedFrameB.leftRightMatches3D;

  // Absolute Orientation [Horn] using RANSAC; indices of best inliers and
  // current inliers
  std::vector<size_t> inlierIndicesMax, inlierIndicesTemp;

  // Randomly selected pts and whether they are too close together
  std::vector<Eigen::Vector3d> selectedPointsFrameA, selectedPointsFrameB;
  bool pointsTooCloseInAllIterations = true;

  // Perform RANSAC iterations
  int i = 0;  // RANSAC iterator
  while (i < absoluteOrientationRansacMaxIterations_) {
    // Select 4 random points
    int pointIndex1 = rand() % numMatchesAtoB;
    int pointIndex2 = rand() % numMatchesAtoB;
    int pointIndex3 = rand() % numMatchesAtoB;
    int pointIndex4 = rand() % numMatchesAtoB;

    // If indices are not unique, try again
    if (pointIndex1 == pointIndex2 || pointIndex1 == pointIndex3 ||
        pointIndex1 == pointIndex4 || pointIndex2 == pointIndex3 ||
        pointIndex2 == pointIndex4 || pointIndex3 == pointIndex4) {
      continue;
    }
    // Count iterations with unique indices
    i++;

    // Clear previous points
    selectedPointsFrameA.clear();
    selectedPointsFrameB.clear();

    // Select random corresponding points
    selectedPointsFrameA.push_back(
        matchedFrameA.leftRightMatches3D[pointIndex1]);
    selectedPointsFrameA.push_back(
        matchedFrameA.leftRightMatches3D[pointIndex2]);
    selectedPointsFrameA.push_back(
        matchedFrameA.leftRightMatches3D[pointIndex3]);
    selectedPointsFrameA.push_back(
        matchedFrameA.leftRightMatches3D[pointIndex4]);

    selectedPointsFrameB.push_back(
        matchedFrameB.leftRightMatches3D[pointIndex1]);
    selectedPointsFrameB.push_back(
        matchedFrameB.leftRightMatches3D[pointIndex2]);
    selectedPointsFrameB.push_back(
        matchedFrameB.leftRightMatches3D[pointIndex3]);
    selectedPointsFrameB.push_back(
        matchedFrameB.leftRightMatches3D[pointIndex4]);

    // Test if any two points are too close, and continue to the next iteration
    // if true This avoids having a similar situation to only having three
    // points, where 180 deg flips are possible
    bool pointsTooClose = false;
    for (size_t j = 0; j < selectedPointsFrameA.size(); j++) {
      for (size_t k = 0; k < selectedPointsFrameA.size(); k++) {
        // Check all different points to make sure they are not too close
        if (j != k) {
          Eigen::Vector3d interpointVecA =
              selectedPointsFrameA[j] - selectedPointsFrameA[k];
          Eigen::Vector3d interpointVecB =
              selectedPointsFrameB[j] - selectedPointsFrameB[k];
          if (interpointVecA.norm() < aboluteOrientationMinInterpointDist_ ||
              interpointVecB.norm() < aboluteOrientationMinInterpointDist_) {
            pointsTooClose = true;
            break;
          }
        }
      }

      if (pointsTooClose) {
        break;
      }
    }

    if (pointsTooClose) {
      if (pointsTooCloseInAllIterations &&
          i == absoluteOrientationRansacMaxIterations_) {
        // If points are too close, try again. If points have been too close in
        // all iterations, return an invalid result
        deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
        deltaPoseAtoB.t << -1, -1, -1;
        if (verbose_) {
          std::cout << "Absolute orientation canceled (points too close in all "
                       "iterations)"
                    << std::endl;
        }
        return;
      }

      continue;

    } else {
      // If points were not too close in this iteration, note this
      pointsTooCloseInAllIterations = false;
    }

    // Get frame change from A to B
    calculateMotionAbsoluteOrientationPoints(
        selectedPointsFrameA, selectedPointsFrameB, deltaPoseAtoB);

    // Determine which of all the matches are inliers (have small reprojection
    // error) according to the calculated AtoB pose change. Here it is developed
    // for a static environment, and a moving camera.
    inlierIndicesTemp.clear();
    Eigen::Matrix3d RBtoA = deltaPoseAtoB.R;
    Eigen::Vector3d tAtoB_A = deltaPoseAtoB.t;
    for (size_t k = 0; k < (size_t)numMatchesAtoB; k++) {
      // Get reprojection error [px]
      Eigen::Vector3d rA_A =
          matchedFrameA.leftRightMatches3D[k];  // pt in frame A in A coords
      Eigen::Vector3d rB_B =
          matchedFrameB.leftRightMatches3D[k];  // pt in frame B in B coords
      Eigen::Vector3d rA_B =
          RBtoA.transpose() * (rA_A - tAtoB_A);  // pt in frame A in B coords

      Eigen::Vector2d reprojectionError =
          featureHandler_->reproject(rB_B) - featureHandler_->reproject(rA_B);

      // If the reprojection error is smaller than the inlier threshold, add to
      // inlier indices
      if (reprojectionError.norm() < absoluteOrientationInlierThreshold_) {
        inlierIndicesTemp.push_back(k);
      }

      // If the metric error in 3D is smaller than the inlier threshold, add to
      // inlier indices
      //			if ((rB_B - rA_B).norm() < 0.005) {
      //				inlierIndicesTemp.push_back(k);
      //			}
    }

    // If there are more inliers this time than the best run, store temporary
    // inlier indices
    if (inlierIndicesTemp.size() > inlierIndicesMax.size()) {
      inlierIndicesMax = inlierIndicesTemp;
    }

    // If all features are inliers, stop RANSAC
    if ((int)inlierIndicesMax.size() == numMatchesAtoB) {
      break;
    }
  }

  // Copy descriptors and keypoints to inlier frames
  inlierFrameA.leftDescriptors = frameA.leftDescriptors;
  inlierFrameA.rightDescriptors = frameA.rightDescriptors;
  inlierFrameA.leftKeypoints = frameA.leftKeypoints;
  inlierFrameA.rightKeypoints = frameA.rightKeypoints;

  inlierFrameB.leftDescriptors = frameB.leftDescriptors;
  inlierFrameB.rightDescriptors = frameB.rightDescriptors;
  inlierFrameB.leftKeypoints = frameB.leftKeypoints;
  inlierFrameB.rightKeypoints = frameB.rightKeypoints;

  // Copy inlier matches, their 3D points and their covariances to inlier frames
  for (size_t l = 0; l < inlierIndicesMax.size(); l++) {
    inlierFrameA.leftRightMatches.push_back(
        matchedFrameA.leftRightMatches[inlierIndicesMax[l]]);
    inlierFrameA.leftRightMatches3D.push_back(
        matchedFrameA.leftRightMatches3D[inlierIndicesMax[l]]);
    inlierFrameA.covLeftRightMatches3D.push_back(
        matchedFrameA.covLeftRightMatches3D[inlierIndicesMax[l]]);

    inlierFrameB.leftRightMatches.push_back(
        matchedFrameB.leftRightMatches[inlierIndicesMax[l]]);
    inlierFrameB.leftRightMatches3D.push_back(
        matchedFrameB.leftRightMatches3D[inlierIndicesMax[l]]);
    inlierFrameB.covLeftRightMatches3D.push_back(
        matchedFrameB.covLeftRightMatches3D[inlierIndicesMax[l]]);
  }

  // If there are not enough inliers, give an invalid result. Otherwise compute
  // the least squares solution using the best inliers
  if ((int)inlierFrameA.leftRightMatches.size() <
      absoluteOrientationMininmumInliers_) {
    deltaPoseAtoB.R = -Eigen::Matrix3d::Identity();
    deltaPoseAtoB.t << -1, -1, -1;
    if (verbose_) {
      std::cout << "Absolute orientation canceled (only "
                << inlierFrameA.leftRightMatches.size() << " inliers)"
                << std::endl;
    }

  } else {
    calculateMotionAbsoluteOrientationPoints(inlierFrameA.leftRightMatches3D,
                                             inlierFrameB.leftRightMatches3D,
                                             deltaPoseAtoB);
  }

  if (verbose_) {
    std::cout
        << "Absolute orientation [inliers / matches] [iterations / time]: ["
        << inlierFrameA.leftRightMatches.size() << " / " << numMatchesAtoB
        << "] [ " << i << " / xx ms]" << std::endl;
    std::cout << "AbsOrientation Motion Estimate: " << deltaPoseAtoB
              << std::endl;
  }
}

// Get the  pose change of the camera from frames A to B (i.e. RBtoA, and
// tAtoB_A) given corresponding 3D points using the closed form solution to
// absolute orientation [Horn1987]. Here it is developed for a static
// environment and a moving camera.
void VisualOdometer::calculateMotionAbsoluteOrientationPoints(
    const std::vector<Eigen::Vector3d>& pointsFrameA,
    const std::vector<Eigen::Vector3d>& pointsFrameB, Pose3D& deltaPoseAtoB) {
  // Find the mean of all the points in each frame
  Eigen::Vector3d frameAMean = Eigen::Vector3d::Zero();
  Eigen::Vector3d frameBMean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < pointsFrameA.size(); i++) {
    frameAMean += pointsFrameA[i];
    frameBMean += pointsFrameB[i];
  }
  frameAMean /= (double)pointsFrameA.size();
  frameBMean /= (double)pointsFrameA.size();

  // Create M matrix for optimization [Horn]
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();  // matrix M from Horn
  for (size_t j = 0; j < pointsFrameA.size(); j++) {
    Eigen::Vector3d rA_Ap =
        pointsFrameA[j] - frameAMean;  // shifted pt in frame A in A coords
    Eigen::Vector3d rB_Bp =
        pointsFrameB[j] - frameBMean;  // shifted pt in frame B in B coords
    Eigen::Matrix3d dM;                // change in matrix M

    dM << rA_Ap(0) * rB_Bp(0), rA_Ap(0) * rB_Bp(1), rA_Ap(0) * rB_Bp(2),
        rA_Ap(1) * rB_Bp(0), rA_Ap(1) * rB_Bp(1), rA_Ap(1) * rB_Bp(2),
        rA_Ap(2) * rB_Bp(0), rA_Ap(2) * rB_Bp(1), rA_Ap(2) * rB_Bp(2);
    M += dM;  // add dM to matrix M
  }

  // Calculate N = sum_{j} ( qbarmat([rA_Ap_j; 0])' * qmat([rB_Bp_j; 0]') )
  // From Horn, but adapted for q = [qv qs]'
  Eigen::Matrix4d N = Eigen::Matrix4d::Zero();  // create N matrix
  N << M(0, 0) - M(1, 1) - M(2, 2), M(0, 1) + M(1, 0), M(2, 0) + M(0, 2),
      M(1, 2) - M(2, 1), M(0, 1) + M(1, 0), -M(0, 0) + M(1, 1) - M(2, 2),
      M(1, 2) + M(2, 1), M(2, 0) - M(0, 2), M(2, 0) + M(0, 2),
      M(1, 2) + M(2, 1), -M(0, 0) - M(1, 1) + M(2, 2), M(0, 1) - M(1, 0),
      M(1, 2) - M(2, 1), M(2, 0) - M(0, 2), M(0, 1) - M(1, 0),
      M(0, 0) + M(1, 1) + M(2, 2);

  // Create eigenvalues / eigenvectors solver and get the real part of the
  // eigenvalues http://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
  Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(N, true);
  Eigen::Vector4cd eigenValuesComplex = eigenSolver.eigenvalues();
  Eigen::Vector4d eigenValues = eigenValuesComplex.real();

  // Get the index of the maximum eigenvalue and associated eigenvector, which
  // corresponds to q
  size_t maxEigenValueCol =
      (eigenValues(0) > eigenValues(1) && eigenValues(0) > eigenValues(2) &&
       eigenValues(0) > eigenValues(3))
          ? 0
          : ((eigenValues(1) > eigenValues(2) &&
              eigenValues(1) > eigenValues(3))
                 ? 1
                 : ((eigenValues(2) > eigenValues(3)) ? 2 : 3));
  Eigen::Vector4cd qBtoAComplex =
      eigenSolver.eigenvectors().col(maxEigenValueCol);
  Eigen::Vector4d qBtoA = qBtoAComplex.real();
  qBtoA /= qBtoA.norm();

  // Note that in [Horn1987] his convention leads to the point rotation rather
  // than the frame change rotation. Return the rotation matrix RBtoA and
  // translation matrix tAtoB_A
  Eigen::Quaterniond qRet(qBtoA(3), qBtoA(0), qBtoA(1), qBtoA(2));
  deltaPoseAtoB.R = omsci::quat2rot(qRet);
  deltaPoseAtoB.t = frameAMean - deltaPoseAtoB.R * frameBMean;
}

// Get the maximum likelihood estimate for pose change of the camera from A to B
// given corresponding points and their covariance [Adapted from Matthies
// PhD 2.3.2 Appendix B.2, see Setterfield Thesis Proposal]. Return the inlier
// points, the pose change, and the covariance of the estimate.
void VisualOdometer::calculateMotionMaximumLikelihood(
    const omsci::StereoFrame& frameA, const omsci::StereoFrame& frameB,
    omsci::StereoFrame& inlierFrameA, omsci::StereoFrame& inlierFrameB,
    Pose3D& deltaPoseAtoB) {
  // Get the absolute orientation and inlier frames from frame A to frame B.
  // This will eliminate features that do not pass the rigidity check, and give
  // an initialization pose for the maximum likelihood estimate.
  calculateMotionAbsoluteOrientation(frameA, frameB, inlierFrameA, inlierFrameB,
                                     deltaPoseAtoB);

  // If no valid absolute orientation was found, return the invalid results
  if (deltaPoseAtoB.R.determinant() == -1) {
    deltaPoseAtoB.cov = Eigen::Matrix<double, 6, 6>::Zero();
    return;
  }

  // Initialize using absolute orientation results
  Eigen::Matrix3d RAtoB = deltaPoseAtoB.R.transpose();  // rot from A to B
  Eigen::Vector3d tAtoB_B =
      deltaPoseAtoB.R * deltaPoseAtoB.t;  // trans from A to B in B coords

  // Initialize delTheta, the so(3) change in rotation solved for at each
  // iteration
  Eigen::Vector3d delTheta;
  delTheta << -1, -1, -1;

  // Initialize sum terms required for solving delTheta = (A - B*inv(C)*B')\(D -
  // B*inv(C)*E)
  Eigen::Matrix3d A, B, C, C_inv;
  Eigen::Vector3d D, E;

  // Initialize sum term required for solving cov([delTheta delT]') = inv(F)
  Eigen::Matrix<double, 6, 6> F;

  // Solve the linearized problem until the orientation change is small
  int iterations = 0;
  while (delTheta.norm() > maxLikelihoodDelThetaThreshold_ &&
         iterations < maxLikelihoodMaxIterations_) {
    // Reset sum terms to zero
    A = Eigen::Matrix3d::Zero();
    B = Eigen::Matrix3d::Zero();
    C = Eigen::Matrix3d::Zero();
    D = Eigen::Vector3d::Zero();
    E = Eigen::Vector3d::Zero();
    F = Eigen::Matrix<double, 6, 6>::Zero();

    // Loop through all features in the inlier frames and get the necessary
    // matrices
    for (int j = 0; j < (int)inlierFrameA.leftRightMatches.size(); j++) {
      Eigen::Vector3d rA_A, rB_B, Q_j;
      Eigen::Matrix3d rA_Ax, R_j, R_j_inv, J_j;
      Eigen::Matrix<double, 3, 6> H_j;

      rA_A =
          inlierFrameA.leftRightMatches3D[j];  // points in frame A in A coords
      rB_B =
          inlierFrameB.leftRightMatches3D[j];  // points in frame B in B coords

      // Covariance of point position error (inverse of this used for weighting)
      R_j = inlierFrameB.covLeftRightMatches3D[j] +
            RAtoB * inlierFrameA.covLeftRightMatches3D[j] * RAtoB.transpose();

      Q_j = rB_B - RAtoB * rA_A + tAtoB_B;  // Q_j term
      rA_Ax = omsci::skew(rA_A);            // skew symmetric matrix from rA_A
      J_j = -RAtoB * rA_Ax;  // lin. sensitivity of predicted rB_B to delTheta
      H_j.block(0, 0, 3, 3) = J_j;  // H_j term
      H_j.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

      R_j_inv = R_j.inverse();  // inverse of covariance matrix

      // Calculate the sum terms
      A = A + J_j.transpose() * R_j_inv * J_j;
      B = B + J_j.transpose() * R_j_inv;
      C = C + R_j_inv;
      D = D + J_j.transpose() * R_j_inv * Q_j;
      E = E + R_j_inv * Q_j;
      F = F + H_j.transpose() * R_j_inv * H_j;
    }

    // Solve for the so(3) change in linearized rotation
    C_inv = C.inverse();
    delTheta = (A - B * C_inv * B.transpose()).inverse() * (D - B * C_inv * E);

    // Update the rotation matrix
    RAtoB = RAtoB * (omsci::skew(delTheta)).exp();

    // Change in translation
    // TODO: can expmap of delTheta be incorporated into this update?
    Eigen::Vector3d delT = C_inv * (E - B.transpose() * delTheta);
    tAtoB_B = tAtoB_B - delT;

    // Increment iteration count
    iterations++;
  }

  // Form the pose PBtoA = [RAtoB tBtoA_B; 0 0 0 1]
  Pose3D PBtoA(RAtoB, -tAtoB_B);

  // Store the rotation from B to A
  Eigen::Matrix3d RBtoA = RAtoB.transpose();

  // updated - need to test
  // Get the found covariance, that is the covariance of RAtoB and tAtoB_B and
  // make it valid for PBtoA (see logbook #3 pp 149)
  Eigen::Matrix<double, 6, 6> covFound = F.inverse();
  PBtoA.cov.block(0, 0, 3, 3) = covFound.block(0, 0, 3, 3);
  PBtoA.cov.block(3, 3, 3, 3) =
      RBtoA * covFound.block(3, 3, 3, 3) * RBtoA.transpose();
  PBtoA.cov.block(0, 3, 3, 3) = covFound.block(0, 3, 3, 3) * RBtoA.transpose();
  PBtoA.cov.block(3, 0, 3, 3) = PBtoA.cov.block(0, 3, 3, 3).transpose();

  // Invert the pose to find PAtoB = [RBtoA tAtoB_A; 0 0 0 1] with proper
  // covariance
  deltaPoseAtoB = PBtoA.inverse();

  /* old - known to work
  deltaPoseAtoB.R 		= RBtoA;

  // Translation from A to B in A coordinates and covariance (expected value of
  // [delTheta RBtoA*delT] [delTheta RBtoA*delT]'). Note that the uncertainty in
  rotation is
  // not altered during the inversion, but the uncertainty in translation is.
  deltaPoseAtoB.t   				 = RBtoA * tAtoB_B;
  deltaPoseAtoB.cov 				 = F.inverse();
  deltaPoseAtoB.cov.block(3,3,3,3) = RBtoA * deltaPoseAtoB.cov.block(3,3,3,3) *
  RBtoA.transpose();
  */

  if (verbose_) {
    std::cout << "Maximum Likelihood [iterations / time]: [" << iterations
              << " / xx ms]" << std::endl;
    std::cout << "MaxLikelihood Motion Estimate: " << deltaPoseAtoB
              << std::endl;
  }
}

}  // namespace sph
