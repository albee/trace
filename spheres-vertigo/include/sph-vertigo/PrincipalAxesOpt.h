/**
 * @file PrincipalAxesOpt.h
 * @brief Object to estimate principal axes of rigid body.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_PRINCIPALAXESOPT_H_
#define SPH_VERTIGO_PRINCIPALAXESOPT_H_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

//#include "cpp-toolbox/GeometryUtils.h"
#include "sph-vertigo/CommonTypes.h"
#include "sph-vertigo/Conics.h"
#include "sph-vertigo/GeometryUtils.h"
#include "polhode_alignment/geometry_utils.h"

namespace sph {

/// Convenient shorthand for a conic fit with specified plane.
template <typename T>
using PlaneFit = std::pair<ConicFit<T>, int>;

/**
 * @brief Finds optimal principal axes alignment from a polhode in random frame.
 */
template <typename T>
class PrincipalAxesOpt {
 public:
  /**
   * @brief Default constructor.
   * @param omegaB_G  3xN matrix with N angular velocity measurement in frame G.
   */
  explicit PrincipalAxesOpt(const Matrix3X<T>& omegaB_G);

  /**
   * @brief Get the best conic fits in each plane given an optimal rotation R.
   * @param[in] R  Optimal rotation from rando frame G to principal axes B.
   * @return Tuple with best fits on XY, XZ, and YZ planes, respectively.
   */
  std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>> GetBestConicFits(
      const Matrix3<T>& R) const;

  /**
   * @brief Get the best constrained conic fits.
   * @param[in] R  Optimal rotation from rando frame G to principal axes B.
   * @return Vector with best fits on XY, XZ, and YZ planes, with the `int`
   * denoting the corresponding plane (1: XY, 2: XZ, 3: YZ).
   */
  std::vector<std::pair<ConicFit<T>, int>> GetBestConicFitsConstrained(
      const Matrix3<T>& R) const;

  /**
   * @brief Find body frame from optimized elliptic frame E.
   * @param[in] R_EG  Optimal rotation from geometric frame G to elliptic E.
   * @return The orientation of body frame wrt geometric frame R_GB.
   */
  Matrix3<T> FindBodyFrame(const Matrix3<T>& R_EG);

  /**
   * @return The `cnHat_E` and `dnHat_E` axes, respectively in tuple.
   */
  std::tuple<Vector3<T>, Vector3<T>> GetHypAxes(
      const Matrix3<T>& R_EG, const std::vector<PlaneFit<T>>& fits) const;

 private:
  inline bool ValueInVec(const int val, const Eigen::Vector3i& vec) const {
    for (int i = 0; i < vec.size(); i++) {
      if (val == vec[i]) return true;
    }
    return false;
  }

  Matrix3X<T> omegaB_G_;  ///< Angular velocity measurement in frame G.
  size_t N_;              ///< Number of measurements.
  ConicFit<T> xyUncFit_, xzUncFit_, yzUncFit_;     ///< Unconstrained fits.
  std::vector<PlaneFit<T>> fits_;                  ///< Constrained fits.
  ConicFit<T> xyBestFit_, xzBestFit_, yzBestFit_;  ///< Best fits.
  // NOTE: currently supporting only tri-axial symmetry.
  // InertiaSymmetry symmetry_ = InertiaSymmetry::TA;  ///< TA, AS1, AS2, FS.
  EnergyState energy_;  ///< HE or LE.
  const Eigen::MatrixXi planeAxes_ =
      (Eigen::MatrixXi(2, 3) << 1, 1, 2, 2, 3, 3).finished();
};

}  // namespace sph

#include "PrincipalAxesOpt.tpp"

#endif  // SPH_VERTIGO_PRINCIPALAXESOPT_H_
