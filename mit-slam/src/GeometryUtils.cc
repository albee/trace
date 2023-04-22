/**
 * @file GeometryUtils.cpp
 * @brief Common useful functions for geometric properties.
 * @date Dec 30, 2020
 * @author tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit_slam/GeometryUtils.h"

namespace mit_slam {

Eigen::VectorXf SampleNormalVector(const int dim) {
  // NOLINTNEXTLINE
  static std::mt19937 gen{std::random_device{}()};
  static std::normal_distribution<float> dist;
  return Eigen::VectorXf{dim}.unaryExpr([](float x) { return dist(gen); });
}

Eigen::Matrix3f q2dcm(const float x, const float y, const float z, const float w) {
  Eigen::Matrix3f rot;
  // clang-format off
  rot << 1 - 2*pow(y, 2) - 2*pow(z, 2), 2*x*y - 2*z*w, 2*x*z + 2*y*w,
         2*x*y + 2*z*w, 1 - 2*pow(x, 2) - 2*pow(z, 2), 2*y*z - 2*x*w,
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*pow(x, 2) - 2*pow(y, 2);
  // clang-format on
  return rot;

}

Eigen::Vector4f qmult(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2)
{
    Eigen::Vector4f q_prod;
    float x1 = q1(0); float y1 = q1(1); float z1 = q1(2); float w1 = q1(3);
    float x2 = q2(0); float y2 = q2(1); float z2 = q2(2); float w2 = q2(3);
    q_prod(0) =  x1*w2 + y1*z2 - z1*y2 + w1*x2;
    q_prod(1) = -x1*z2 + y1*w2 + z1*x2 + w1*y2;
    q_prod(2) =  x1*y2 - y1*x2 + z1*w2 + w1*z2;
    q_prod(3) = -x1*x2 - y1*y2 - z1*z2 + w1*w2;
    return q_prod.normalized();
}

Eigen::Vector4f q_error(const Eigen::Vector4f &est_quat, const Eigen::Vector4f &truth_quat) {
  Eigen::Vector4f q_err;
  Eigen::Vector4f q_est_conj;
  q_est_conj << -est_quat(0), -est_quat(1), -est_quat(2), est_quat(3);
  q_err = qmult(truth_quat, q_est_conj);
  return q_err;

}

Eigen::Vector4f dcm2q(const Eigen::Matrix3f &dcm) {

    Eigen::Vector4f q;
    double tr = dcm(0,0) + dcm(1,1) + dcm(2,2);
    if(tr > 0)
    {
      double S = sqrt(tr + 1) * 2;
      q(3) = 0.25 * S;
      q(0) = (dcm(2,1) - dcm(1,2)) / S;
      q(1) = (dcm(0,2) - dcm(2,0)) / S;
      q(2) = (dcm(1,0) - dcm(0,1)) / S;

    } else if(dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2))
    {
      double S = sqrt(1.0 + dcm(0,0) - dcm(1,1) - dcm(2,2)) * 2;
      q(3) = (dcm(2,1) - dcm(1,2)) / S;
      q(0) = 0.25 * S;
      q(1) = (dcm(0,1) + dcm(1,0)) / S;
      q(2) = (dcm(0,2) + dcm(2,0)) / S;

    } else if(dcm(1,1) > dcm(2,2))
    {
      double S = sqrt(1.0 + dcm(1,1) - dcm(0,0) - dcm(2,2)) * 2;
      q(3) = (dcm(0,2) - dcm(2,0)) / S;
      q(0) = (dcm(0,1) + dcm(1,0)) / S;
      q(1) = 0.25 * S;
      q(2) = (dcm(1,2) + dcm(2,1)) / S;

    } else
    {
      double S = sqrt(1.0 + dcm(2,2) - dcm(0,0) - dcm(1,1)) * 2;
      q(3) = (dcm(1,0) - dcm(0,1)) / S;
      q(0) = (dcm(0,2) + dcm(2,0)) / S;
      q(1) = (dcm(1,2) + dcm(2,1)) / S;
      q(2) = 0.25 * S;
    }
    return q;

}

Eigen::Matrix4f odomAdjustGround(const Eigen::Matrix4f & tfm_input) {
  Eigen::Matrix4f tfm_output = tfm_input;
  Eigen::Vector4f q_input = dcm2q(tfm_input.block(0, 0, 3, 3));
  Eigen::Vector3f q_input_3d;
  q_input_3d << q_input(0), q_input(1), q_input(2);
  float angle = 2.0 * asin(q_input_3d.norm());
  Eigen::Vector4f q_output;
  q_output << 0.0, 0.0, sin(angle / 2.0), cos(angle / 2.0);
  tfm_output.block(0, 0, 3, 3) = q2dcm(q_output(0), q_output(1), q_output(2), q_output(3)).transpose();
  tfm_output(2,3) = 0.0;
  return tfm_output;
}

// eliminate NaNs, enforce last row of T matrix to be [0 0 0 1]
Eigen::Matrix4f fixNumerics(const Eigen::Matrix4f &tfm_input) {
  Eigen::Matrix4f tfm_output = tfm_input;
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
        if (std::isnan(tfm_output(i,j)) || ((i == 3) && (j == 0 || j == 1 || j == 2))) {
            tfm_output(i,j) = 0.0;
        }
    }
  }
  return tfm_output;
}

// Calculate angular velocity
Eigen::Vector3f ComputeAngularVelocity(const Eigen::Matrix3f &R1,
                                       const Eigen::Matrix3f &R2,
                                       const double deltaT) {
  //Eigen::Matrix3f deltaR = R1.transpose() * R2;
  Eigen::Matrix3f deltaR = R2.transpose() * R1;
  Eigen::Vector3f deltaTheta = Log(deltaR);
  return (1.0/deltaT)*deltaTheta;

} // end of ComputeAngularVelocity

// Log function
Eigen::Vector3f Log(const Eigen::Matrix3f &R) {

   if (R.isApprox(Eigen::Matrix3f::Identity()))
    return Eigen::Vector3f::Zero();

   double mag = std::acos((R.trace() - 1.0) /2.0);
   Eigen::Vector3f theta = (mag / (2 * std::sin(mag))) * Vee(R - R.transpose());
   return theta;

} // end of Log function

// Vector function
Eigen::Vector3f Vee(const Eigen::Matrix3f &M){

   Eigen::Vector3f vec{-M(1,2), M(0,2), -M(0,1)};
   return vec;

} // end of Vector function

}  // namespace slam
