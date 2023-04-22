/**
 * @file    BetweenMomentumFactor.h
 * @brief   Random walk factor to constrain angular momentum.
 * @author  Tonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/BetweenMomentumFactor.h"

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
BetweenMomentumFactor::BetweenMomentumFactor(const Key& h_i, const Key& h_j,
                                             const Key& bh_i, const Key& bh_j,
                                             const SharedNoiseModel& model)
    : Base(model, h_i, h_j, bh_i, bh_j) {}

/* ************************************************************************* */
Vector BetweenMomentumFactor::evaluateError(
    const Vector3& h_i, const Vector3& h_j, const Vector3& bh_i,
    const Vector3& bh_j, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3,
    boost::optional<Matrix&> H4) const {
  // Get the magnitude error with random walk model.
  Point3 h_err_vec = Point3(h_j - (h_i + bh_i));

  // Get the Jacobian and the error for the angular momentum vector difference.
  double error;         // Delta angular momentum magnitude.
  Matrix H_err_hDelta;  // Jacobian of the magnitude error wrt full delta h vec.
  if (H1 || H2 || H3) {
    error = h_err_vec.norm(H_err_hDelta);
  } else {
    error = h_err_vec.norm();
  }

  // Get the bias error.
  Vector3 bias_error = bh_j - bh_i;

  // Assemble the Jacobians if required.
  if (H1) {  // wrt h_i.
    *H1 = Matrix::Zero(4, 3);
    H1->block(0, 0, 1, 3) = H_err_hDelta;
  }
  if (H2) {  // wrt h_j.
    *H2 = Matrix::Zero(4, 3);
    H2->block(0, 0, 1, 3) = -H_err_hDelta;
  }
  if (H3) {  // wrt bh_i.
    *H3 = Matrix::Zero(4, 3);
    H3->block(0, 0, 1, 3) = -H_err_hDelta;
    H3->block(1, 0, 3, 3) = -Matrix::Identity(3, 3);
  }
  if (H4) {  // wrt bh_j.
    *H4 = Matrix::Zero(4, 3);
    H4->block(1, 0, 3, 3) = Matrix::Identity(3, 3);
  }

  // Assemble the error.
  Vector4 full_err;
  full_err(0) = error;
  full_err.segment(1, 3) = bias_error;

  return full_err;
}

/* ************************************************************************* */
void BetweenMomentumFactor::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  // if (noiseModel_) noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool BetweenMomentumFactor::equals(const BetweenMomentumFactor& factor,
                                   double tol) const {
  return Base::equals(factor);
}

}  // end namespace gtsam
