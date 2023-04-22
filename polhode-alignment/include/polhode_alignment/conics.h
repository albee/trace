/**
 * @file Conics.h
 * @brief Functions related to conic sections.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#ifndef POLHODE_ALIGNMENT_CONICS_H_
#define POLHODE_ALIGNMENT_CONICS_H_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
//#include <math.h>

namespace pao {

/// Type of conic available for least squares fitting.
enum class ConicType { kEllipse, kHyperbola };
inline std::string ConicTypeString(const ConicType& t) {
  if (t == ConicType::kEllipse)
    return std::string("Ellipse");
  else
    return std::string("Hyperbola");
}

/// Direction of the conic.
enum class ConicDirection { kXY, kYX };
inline std::string ConicDirectionString(const ConicDirection& d) {
  if (d == ConicDirection::kXY)
    return std::string("XY");
  else
    return std::string("YX");
}

/// Structure holding a parametric conic's parameters, representing the
/// equation:   Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0.
template <typename T>
struct ParametricConic {
  T A, B, C, D, E, F;  ///< Coefficients of parametric equation.
};

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

}  // namespace pao

#include "conics.tpp"

#endif  // POLHODE_ALIGNMENT_CONICS_H_
