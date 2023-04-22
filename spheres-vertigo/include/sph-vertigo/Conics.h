/**
 * @file Conics.h
 * @brief Functions related to conic sections.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_CONICS_H_
#define SPH_VERTIGO_CONICS_H_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

// Shorten Eigen's typedefs for convenience.
template <typename T>
using VectorType = typename Eigen::GeneralizedEigenSolver<
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>::VectorType;
template <typename T>
using ComplexVectorType = typename Eigen::GeneralizedEigenSolver<
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>::ComplexVectorType;
template <typename T>
using EigenvectorsType = typename Eigen::GeneralizedEigenSolver<
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>::EigenvectorsType;

// clang-format off
template <typename T>
static const Eigen::Matrix<T, 3, 3> CeigConst = (Eigen::Matrix<T, 3, 3>() <<
                                                 T(0.0), T(0.5), T(0.0),
                                                 T(0.5), T(0.0), T(0.0),
                                                 T(0.0), T(0.0), T(0.0))
                                                 .finished();
// clang-format on

/// Structure holding a parametric conic's parameters.
template <typename T>
struct ParametricConic {
  T A, B, C, D, E, F;  ///< Coefficients of parametric equation.
  ParametricConic(const T A = T(0.0), const T B = T(0.0), const T C = T(0.0),
                  const T D = T(0.0), const T E = T(0.0), const T F = T(0.0))
      : A(A), B(B), C(C), D(D), E(E), F(F) {}

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const ParametricConic<T>& c) {
    os << "Conic coefficients: ---\n"
       << " K: [A B C D E F] = [" << c.A << " " << c.B << " " << c.C << " "
       << c.D << " " << c.E << " " << c.F << "]" << std::endl;
    return os;
  }
};

/// Comparison operator for `ParametricConic`.
template <typename T>
inline bool operator==(const ParametricConic<T>& lhs,
                       const ParametricConic<T>& rhs) {
  return (lhs.A == rhs.A && lhs.B == rhs.B && lhs.C == rhs.C &&
          lhs.D == rhs.D && lhs.E == rhs.E && lhs.F == rhs.F);
}

/// Structure holding a canonical conic's description parameters.
template <typename T>
struct ConicCanonicalGeometry {
  T aSemi;
  T bSemi;
  T ecc;
  ConicType type;
  ConicDirection dir;

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const ConicCanonicalGeometry<T>& c) {
    os << "Conic geometry: ---\n"
       << " aSemi: " << c.aSemi << std::endl
       << " bSemi: " << c.bSemi << std::endl
       << " ecc: " << c.ecc << std::endl
       << " type: " << ConicTypeString(c.type) << std::endl
       << " direction: " << ConicDirectionString(c.dir) << std::endl;
    return os;
  }
};

/// Comparison operator for `ConicCanonicalGeometry`.
template <typename T>
inline bool operator==(const ConicCanonicalGeometry<T>& lhs,
                       const ConicCanonicalGeometry<T>& rhs) {
  return (lhs.aSemi == rhs.aSemi && lhs.bSemi == rhs.bSemi &&
          lhs.ecc == rhs.ecc && lhs.type == rhs.type && lhs.dir == rhs.dir);
}

/**
 * @brief Class describing a parametric conic K and its canonical version.
 *
 * Represents the conic `Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0`, in its actual
 * translated and rotated frame G. Also finds rotation and translation from
 * canonical frame to actual parametric frame A.
 */
template <typename T>
class Conic {
 public:
  /**
   * @brief Constructor using all conic parameters.
   */
  Conic(const T A, const T B, const T C, const T D, const T E, const T F);

  // Getters for each coefficient.
  inline T A() const { return A_; }
  inline T B() const { return B_; }
  inline T C() const { return C_; }
  inline T D() const { return D_; }
  inline T E() const { return E_; }
  inline T F() const { return F_; }
  inline T a() const { return geoC_.aSemi; }
  inline T b() const { return geoC_.bSemi; }
  inline T ecc() const { return geoC_.ecc; }
  inline ConicType Type() const { return geoC_.type; }
  inline ConicDirection Direction() const { return geoC_.dir; }
  inline ParametricConic<T> K() const { return K_; }
  inline ParametricConic<T> Kc() const { return Kc_; }
  inline ConicCanonicalGeometry<T> Geo() const { return geoC_; }
  inline Eigen::Matrix<T, 3, 3> T_AC() const { return T_AC_; }

  /**
   * @brief Obtain (x,y) points from canonical conic for a set of angles.
   * @param[in] angles  Vector of angles at which to evaluate the conic.
   * @return Matrix with [x,y]^T conic points.
   */
  Eigen::Matrix<T, 2, Eigen::Dynamic> EvaluateCanonical(
      const std::vector<T>& angles);

  /**
   * @brief Overload to use Eigen vector instead of individual `T`s.
   * @param[in] angles  Vector of angles at which to evaluate the conic.
   * @return Matrix with [x,y]^T conic points.
   */
  Eigen::Matrix<T, 2, Eigen::Dynamic> EvaluateCanonical(
      const Eigen::Matrix<T, Eigen::Dynamic, 1>& angles);

  /**
   * @brief Obtain (x,y) points from actual parametric conic.
   * @param[in] angles  Vector of angles at which to evaluate the conic.
   * @return Matrix with [x,y]^T conic points.
   */
  Eigen::Matrix<T, 2, Eigen::Dynamic> Evaluate(const std::vector<T>& angles);

  /**
   * @brief Overload to use Eigen vector instead of individual `T`s.
   * @param[in] angles  Vector of angles at which to evaluate the conic.
   * @return Matrix with [x,y]^T conic points.
   */
  Eigen::Matrix<T, 2, Eigen::Dynamic> Evaluate(
      const Eigen::Matrix<T, Eigen::Dynamic, 1>& angles);

  /**
   * @brief Convenience function for print out.
   */
  inline friend std::ostream& operator<<(std::ostream& os, const Conic<T>& c) {
    os << "Conic ===========\n"
       << "Parametric: " << std::endl
       << c.K() << "Canonical: " << std::endl
       << c.Kc() << c.Geo() << "Transform T_AC:" << std::endl
       << c.T_AC() << std::endl
       << "=================" << std::endl;
    return os;
  }

 private:
  T A_, B_, C_, D_, E_, F_;  ///< Coefficients of parametric equation.
  // T aSemi_, bSemi_;          ///< Conic's semi axes.
  // T ecc_;                    ///< Eccentricity of conic.
  ParametricConic<T> K_;   ///< Bundled parametric coefficients.
  ParametricConic<T> Kc_;  ///< Bundled canonical representation.
  // ConicType type_;                ///< Type of conic.
  // ConicDirection dir_;            ///< Direction of the conic.
  ConicCanonicalGeometry<T> geoC_;  ///< Canonical conic parameters.
  Eigen::Matrix<T, 3, 3> T_AC_;  ///< Pose of canonical wrt parametric frame A.

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Bundle with canonical conic parameters, residual, and conic type.
 */
template <typename T>
struct ConicFit {
  Conic<T> c = Conic<T>(T(1), T(0), T(2), T(0), T(0),
                     T(-1));  ///< Full conic, dummy values.
  T residual = T(-1);         // Best fit cost, i.e., min(abs(lambda)).
  T residual_square = T(-1);
  ParametricConic<T> K;
  Eigen::Matrix<T, 3, 1> eigv{T(-1), T(-1),
                              T(-1)};  // Raw, unit eigenvector before scaling.
  Eigen::Matrix<T, 3, 1> eigvec{T(-1), T(-1),
                              T(-1)};  // Raw, unit eigenvector before scaling.

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const ConicFit<T>& r) {
    os << "Conic fit result =======\n"
       << "Residual: " << r.residual << std::endl
       << "Eigenvec: " << r.eigv.transpose() << std::endl
       << r.c << std::endl;
    return os;
  }
};

/**
 * @brief Find best canonical ellipse and hyperbola fits to the provided data.
 * @param[in] pts  2xN matrix with (x,y) plane points.
 * @return Pair of fits, first for ellipse, second for hyperbola.
 */
// std::pair<ConicFit<double>, ConicFit<double>> FitConicCanonical(
//     const Eigen::MatrixXd& pts);
/**
 * @brief Same as `FitConicCanonical`, only with a templated type.
 */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> FitConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts);

template <typename T>
ConicFit<T> BuildConicFit(const Eigen::Matrix<T, 3, 3>& D, const T& lambda,
                          const T& alpha11, const T& alpha21);

/**
 * @brief Find best canonical fit between ellipse and hyperbola to data.
 * @param[in] pts  2xN matrix with (x,y) plane points.
 * @return Best canonical conic fit.
 */
template <typename T>
ConicFit<T> FitBestConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts);

/**
 * @brief Find parametric conic equation parameters for corresponding canonical.
 * @param[in]  K    The parametric conic in question.
 * @param[out] tfm  SE(2) pose of canonical frame wrt actual parametric frame.
 * @return The bundled conic parameters for canonical representation.
 */
template <typename T>
ParametricConic<T> FindCanonical(const ParametricConic<T>& K,
                                 Eigen::Matrix<T, 3, 3>* tfm);

/**
 * @brief Find the geometric parameters describing the canonical conic.
 * @param[in] K  The canonical parametric conic in question.
 * @return Geometric parameters of canonical conic.
 */
template <typename T>
ConicCanonicalGeometry<T> GetCanonicalGeometry(const ParametricConic<T>& Kc);

/**
 * @brief Check whether conic argument is canonical.
 * @param[in] Kc  Bundled parametric conic to be tested.
 * @return Whether conic is unrotated and centered at the origin.
 */
template <typename T>
bool IsCanonical(const ParametricConic<T>& Kc);

/**
 * @brief Extract conic fits from generalized eigenvalue problem solution.
 * @param eigenvecs  Matrix with resulting eigenvectors.
 * @param alphas     Complex number vector with eigenvalues' numerators.
 * @param betas      Real-valued vector with eigenvalues' denominator.
 *
 * The resulting eigenvalues can then be recovered as alphas/.betas.
 */
// std::pair<ConicFit<double>, ConicFit<double>> ParseEigenPair(
//     const EigenvectorsType<double>& eigenvecs,
//     const ComplexVectorType<double>& alphas, const VectorType<double>&
//     betas);
/**
 * @brief Same as `ParseEigenPair`, only with a templated type.
 */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> ParseEigenPair(
    const EigenvectorsType<T>& eigenvecs, const ComplexVectorType<T>& alphas,
    const VectorType<T>& betas);

/**
 * @brief Get the best conic fits in each plane given an optimal rotation R_EG.
 * @param[in] R_EG  Optimal rotation from rando frame G to hyperbolic axis E.
 * @param[in] G_omegaB  Angular velocity matrix 3xN in rando frame G.
 * @return Tuple with best fits on XY, XZ, and YZ planes, respectively.
 */
template <typename T>
std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>> GetBestConicFits(
    const Matrix3<T>& R_EG, const Matrix3X<T>& G_omegaB);

}  // namespace sph

#include "Conics.tpp"

#endif  // SPH_VERTIGO_CONICS_H_
