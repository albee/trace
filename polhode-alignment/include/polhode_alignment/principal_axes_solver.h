/**
 * @file principal_axes_solver.h
 * @brief Ceres-based solver for finding principal axes orientation.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */
#ifndef POLHODE_ALIGNMENT_PRINCIPAL_AXES_SOLVER_H_
#define POLHODE_ALIGNMENT_PRINCIPAL_AXES_SOLVER_H_

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/types.h>

#include <Eigen/Core>

#include "polhode_alignment/conic_fitting.h"
#include "polhode_alignment/geometry_utils.h"

namespace pao {

class PrincipalAxesAlignmentCost {
 public:
  explicit PrincipalAxesAlignmentCost(const Eigen::MatrixXd G_omegaB)
      : G_omegaB_(G_omegaB) {}

  template <typename T>
  bool operator()(const T* const E_quat_G_ptr, T* residual) const {
    // Get the proposed rotation matrix from the rotation parameterization.
    Eigen::Map<const Eigen::Quaternion<T>> E_quat_G(E_quat_G_ptr);
    Eigen::Matrix<T, 3, 3> E_rot_G = E_quat_G.toRotationMatrix().transpose();

    // Find best conic fist in E planes [1 2], [2 3], and [1 3].
    ConicFit<T> fit_xy, fit_xz, fit_yz;
    std::tie(fit_xy, fit_xz, fit_yz) =
        GetBestConicFits<T>(E_rot_G, G_omegaB_.cast<T>());

    // Const is the sum of the deviations from the conic equation
    //   a*x^2 + b*y^2 + f = 0,
    // where each plane has the best conic fit possible.
    residual[0] = T(fit_xy.residual_square) + T(fit_xz.residual_square) +
                  T(fit_yz.residual_square);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::MatrixXd G_omegaB) {
    return (new ceres::AutoDiffCostFunction<PrincipalAxesAlignmentCost, 1, 4>(
        new PrincipalAxesAlignmentCost(G_omegaB)));
  }

 private:
  Eigen::MatrixXd G_omegaB_;  ///< Angular velocity measurements in frame G.
};

class PrincipalAxesAlignmentAACost {
 public:
  explicit PrincipalAxesAlignmentAACost(const Eigen::MatrixXd G_omegaB)
      : G_omegaB_(G_omegaB) {}

  template <typename T>
  bool operator()(const T* const E_aa_G_ptr, T* residual) const {
    // Get the proposed rotation matrix from the rotation parameterization.
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> E_aa_G(E_aa_G_ptr);
    Eigen::Matrix<T, 3, 3> E_rot_G = ExpMapRot<T>(E_aa_G);

    // Find best conic fist in E planes [1 2], [2 3], and [1 3].
    ConicFit<T> fit_xy, fit_xz, fit_yz;
    std::tie(fit_xy, fit_xz, fit_yz) =
        GetBestConicFits<T>(E_rot_G, G_omegaB_.cast<T>());

    // Const is the sum of the deviations from the conic equation
    //   a*x^2 + b*y^2 + f = 0,
    // where each plane has the best conic fit possible.
    residual[0] = T(fit_xy.residual_square) + T(fit_xz.residual_square) +
                  T(fit_yz.residual_square);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::MatrixXd G_omegaB) {
    return (new ceres::AutoDiffCostFunction<PrincipalAxesAlignmentAACost, 1, 3>(
        new PrincipalAxesAlignmentAACost(G_omegaB)));
  }

 private:
  Eigen::MatrixXd G_omegaB_;  ///< Angular velocity measurements in frame G.
};

/*
struct PrincipalAxesCostFunctor {
  PrincipalAxesCostFunctor(const Eigen::MatrixXd& G_omegaB)
      : G_omegaB_(G_omegaB) {}

  bool operator()(const double* const E_quat_G_ptr, double* residual) const {
    // Get the proposed rotation matrix from the rotation parameterization.
    Eigen::Map<const Eigen::Quaterniond> E_quat_G(E_quat_G_ptr);
    Eigen::Matrix3d E_rot_G(E_quat_G);

    // Find best conic fist in E planes [1 2], [2 3], and [1 3].
    ConicFit<double> fit_xy, fit_xz, fit_yz;
    std::tie(fit_xy, fit_xz, fit_yz) =
        sph::GetBestConicFits<double>(E_rot_G, G_omegaB_);

    // Const is the sum of the deciations from the conic equation
    //   a*x^2 + b*y^2 + f = 0,
    // where each plane has the best conic fit possible.
    residual[0] = fit_xy.residual + fit_xz.residual + fit_yz.residual;

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::MatrixXd& G_omegaB) {
    return (new ceres::NumericDiffCostFunction<PrincipalAxesCostFunctor,
                                               ceres::RIDDERS, 1, 4>(
        new PrincipalAxesCostFunctor(G_omegaB)));
  }

  Eigen::MatrixXd G_omegaB_;  ///< Angular velocity measurements in frame G.
};
*/

}  // namespace pao

#endif  // POLHODE_ALIGNMENT_PRINCIPAL_AXES_SOLVER_H_
