/**
 * @file CommonTypes.cpp
 * @brief Custom SPHERES-related types.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Adapted by Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/CommonTypes.h"

namespace sph {}  // namespace sph

namespace omsci {

// Create a skew symmetric cross-product matrix
Eigen::Matrix3d skew(const Eigen::Vector3d vec) {
  Eigen::Matrix3d skewMatrix;
  skewMatrix << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skewMatrix;
}

// Convert from quaternion to rotation matrix, using SPHERES conventions
Eigen::Matrix3d quat2rot(const Eigen::Quaterniond quat) {
  return (quat.conjugate()).toRotationMatrix();
}

// Convert from quaternion to rotation matrix, using SPHERES conventions
Eigen::Matrix3d quat2rot(const Eigen::Vector4d quat) {
  Eigen::Quaterniond quatEig(quat(3), quat(0), quat(1), quat(2));
  return (quatEig.conjugate()).toRotationMatrix();
}

// Convert from rotation matrix to quaternion, using SPHERES conventions
Eigen::Quaterniond rot2quat(const Eigen::Matrix3d rot) {
  Eigen::Quaterniond quatEig(rot.transpose());
  return quatEig;
}

}  // namespace omsci
