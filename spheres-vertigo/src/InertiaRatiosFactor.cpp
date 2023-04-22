/**
 * @file    InertiaRatiosFactor.h
 * @brief   Estimation of the inertia ratios using conservation of ang momentum.
 * @author  Tonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/InertiaRatiosFactor.h"

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
InertiaRatiosFactor::InertiaRatiosFactor(const Key& J, const Key& h_ij,
                                         const Key& T_GiBi, const Key& T_GjBj,
                                         const Key& T_WBi, const Key& T_WBj,
                                         const SharedNoiseModel& model,
                                         const Rot3& R_BG,
                                         const double deltaT_ij)
    : Base(model, J, h_ij, T_GiBi, T_GjBj, T_WBi, T_WBj),
      R_BG_(R_BG),
      deltaT_ij_(deltaT_ij) {}

/* ************************************************************************* */
Vector InertiaRatiosFactor::evaluateError(
    const Vector2& J, const Vector3& h_ij, const Pose3& T_GiBi,
    const Pose3& T_GjBj, const Pose3& T_WBi, const Pose3& T_WBj,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3, boost::optional<Matrix&> H4,
    boost::optional<Matrix&> H5, boost::optional<Matrix&> H6) const {
  // Get the delta pose of the frame G between time i and j to extract rotation.
  Pose3 T_GiGj = T_GiBi * T_WBi.inverse() * T_WBj * T_GjBj.inverse();
  Vector3 G_omega_ij = (1.0 / deltaT_ij_) * Rot3::Logmap(T_GiGj.rotation());

  // Get the angular velocity "measurement" in principal axes.
  Vector3 B_omega_ij = (R_BG_ * Point3(G_omega_ij)).vector();

  // Jacobians if requested.
  Matrix Z_3x6 = Matrix::Zero(3, 6);
  if (H1) {  // Jacobian wrt inertia ratios J1 and J2.
    Matrix H_err_J = Matrix::Zero(3, 2);
    H_err_J(0, 0) = B_omega_ij(0);  // omega_x.
    H_err_J(1, 1) = B_omega_ij(1);  // omega_y.
    *H1 = H_err_J;
  }
  if (H2) *H2 = Matrix::Identity(3, 3);  // Jacobian wrt  h_ij.
  if (H3) *H3 = Z_3x6;  // Factor conditions upon these variables.
  if (H4) *H4 = Z_3x6;
  if (H5) *H5 = Z_3x6;
  if (H6) *H6 = Z_3x6;

  // Compute the error J*omega - h = 0.
  Matrix Jmat = Matrix::Identity(3, 3);
  Jmat(0, 0) = J(0);  // J1.
  Jmat(1, 1) = J(1);  // J1.
  Vector3 err = Jmat * B_omega_ij - h_ij;

  return err;
}

/* ************************************************************************* */
void InertiaRatiosFactor::print(const std::string& s,
                                const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  // if (noiseModel_) noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool InertiaRatiosFactor::equals(const InertiaRatiosFactor& factor,
                                 double tol) const {
  return Base::equals(factor);
}

}  // end namespace gtsam
