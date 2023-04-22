/**
 * @file geometry_utils.h
 * @brief Utility functions for geometry related stuff.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#ifndef POLHODE_ALIGNMENT_GEOMETRY_UTILS_H_
#define POLHODE_ALIGNMENT_GEOMETRY_UTILS_H_

#include <random>

#include "polhode_alignment/conic_fitting.h"
#include "polhode_alignment/conics.h"

namespace pao {

/* ************************************************************************** */

/// Pretty printing for parametric conics.
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const ParametricConic<T>& c) {
  os << "Conic coefficients: ---\n"
     << " K: [A B C D E F] = [" << c.A << " " << c.B << " " << c.C << " " << c.D
     << " " << c.E << " " << c.F << "]" << std::endl;
  return os;
}

template <typename T>
T sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

inline bool ValueInVec(const int val, const Eigen::Vector3i& vec) {
    for (int i = 0; i < vec.size(); i++) {
      if (val == vec[i]) return true;
    }
    return false;
  }

/// Pretty printing for conic fits.
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const ConicFit<T>& r) {
  os << "Conic fit result =======\n"
     << "Sqrd. residual: " << r.residual_square << std::endl
     << "Eigenvector   : " << r.eigvec.transpose() << std::endl
     << r.K << std::endl;
  return os;
}

/* ************************************************************************** */

/// Comparison operator for `ParametricConic`.
template <typename T>
inline bool operator==(const ParametricConic<T>& lhs,
                       const ParametricConic<T>& rhs) {
  return (lhs.A == rhs.A && lhs.B == rhs.B && lhs.C == rhs.C &&
          lhs.D == rhs.D && lhs.E == rhs.E && lhs.F == rhs.F);
}

/// Comparison operator for `ConicCanonicalGeometry`.
template <typename T>
inline bool operator==(const ConicCanonicalGeometry<T>& lhs,
                       const ConicCanonicalGeometry<T>& rhs) {
  return (lhs.aSemi == rhs.aSemi && lhs.bSemi == rhs.bSemi &&
          lhs.ecc == rhs.ecc && lhs.type == rhs.type && lhs.dir == rhs.dir);
}

/**
 * @brief Sample isotropic Gaussian distribution with common mean value across.
 * @param[in] dim The dimension of the underlying distribution.
 * @param[in] mean Mean value for all dimensions.
 * @param[in] stddev Standard deviation of the underlying distribution.
 * @return `dim`-dimensional sample from a normal distribution.
 */
template <typename ScalarT>
Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> SampleNormalMatrix(
    const int rows, int cols, const ScalarT mean, const ScalarT stddev) {
  // NOLINTNEXTLINE
  std::mt19937 gen{std::random_device{}()};
  std::normal_distribution<ScalarT> norm_dist{mean, stddev};
  //return Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>{rows, cols}
  //    .unaryExpr([&](auto x) { return norm_dist(gen); });
  return Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>{rows, cols}
      .unaryExpr([&](float x) { return norm_dist(gen); });
}


/**
 * @brief Apply the skew (wedge) operator to a 3D vector.
 * @param[in] vec Vector in 3 dimensions.
 * @return Skew symmetric matrix corresponding to vec^.
 */
template <typename ScalarT>
Eigen::Matrix<ScalarT, 3, 3> SkewMat(const Eigen::Matrix<ScalarT, 3, 1> &vec) {
  // Create and allocate memory for skew symmetric matrix.
  Eigen::Matrix<ScalarT, 3, 3> skew_matrix =
    Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, 3);

  // Place the off-diagonal elements.
  skew_matrix(0, 1) = -vec(2, 0);
  skew_matrix(0, 2) = vec(1, 0);
  skew_matrix(1, 0) = vec(2, 0);
  skew_matrix(1, 2) = -vec(0, 0);
  skew_matrix(2, 0) = -vec(1, 0);
  skew_matrix(2, 1) = vec(0, 0);

  return skew_matrix;
}

/**
 * @brief Exponential map operator ExpMapRot : so(3) -> SO(3).
 * @param[in] theta An angle-axis vector, such that theta^ in so(3).
 * @return The corresponding SO(3) rotation matrix.
 *
 * Must ensure that `theta`'s magnitude is upper bounded by pi. Applies the
 * closed form solution of the exponential map:
 * R = I + (sin(theta) / theta) * theta^ + (1 - cos(theta) / theta^2) (theta^)^2
 */
template <typename ScalarT>
Eigen::Matrix<ScalarT, 3, 3> ExpMapRot(
    const Eigen::Matrix<ScalarT, 3, 1>& theta) {
  Eigen::Matrix<ScalarT, 3, 3> I_3 = Eigen::Matrix<ScalarT, 3, 3>::Identity();

  // Get the angle, sine and cosines, and the element of so(3).
  ScalarT angle = theta.norm();         // Rotation angle, in radians.
  if (angle == ScalarT(0)) return I_3;  // Corner case.
  ScalarT c = cos(angle);
  ScalarT s = sin(angle);
  Eigen::Matrix<ScalarT, 3, 3> theta_skew = SkewMat<ScalarT>(theta);

  // Compute the full expression for the exponential map.
  Eigen::Matrix<ScalarT, 3, 3> R;
  R = I_3 + ((s / angle) * theta_skew) +
      (((ScalarT(1) - c) / pow(angle, 2)) * (theta_skew * theta_skew));

  return R;
}

}  // namespace pao

#endif  // POLHODE_ALIGNMENT_GEOMETRY_UTILS_H_
