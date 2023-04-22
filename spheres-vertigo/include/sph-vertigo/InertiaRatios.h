/**
 * @file InertiaRatios.h
 * @brief Manifold corresponding to physically consistent inertia ratios.
 * @date September 05, 2020
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_INERTIARATIOS_H_
#define SPH_VERTIGO_INERTIARATIOS_H_

#include <gtsam/base/concepts.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

namespace sph {

/**
 * @brief Inertia ratios "manifold", GTSAM-style, via orthogonal projection.
 */
struct InertiaRatios {
  double J1;       ///< Major axis ratio Ixx/Izz.
  double J2;       ///< Intermediate axis ratio Iyy/Izz.
  double epsilon;  ///< For comparison purposes.

  /// Possible location areas from R2.
  enum class Region {
    O,    // Feasible region.
    I,    // Open region above (x = y) and above (-x+2 = y).
    II,   // Closed region below (-x+2 = y) and left of (x = 1).
    III,  // Open region between (x = 1) and (x = 2) and below (1 = y).
    IV,   // Closed region right of (x = 2) and below (-x+3 = y).
    V     // Open region below (x - 1 = y) and above (-x+3 = y).
  };

  /**
   * @brief Convenience constructor.
   * @params J1  Major axis ratio Ixx/Izz.
   * @params J2  Intermediate axis ratio Iyy/Izz.
   */
  InertiaRatios(const double J1, const double J2)
      : J1(J1), J2(J2), epsilon(1e-6) {}

  /**
   * @brief Checks to see if point is inside the physically consistent set.
   * @params J1  Major axis ratio Ixx/Izz.
   * @params J2  Intermediate axis ratio Iyy/Izz.
   * @return True if it does not violate the physically consistent constraints.
   */
  bool IsInside(const double J1, const double J2) const;

  /**
   * @brief Determine in which region of the R2 plane the current point is.
   * @params J1  Major axis ratio Ixx/Izz.
   * @params J2  Intermediate axis ratio Iyy/Izz.
   * @return Region enum denoting the region of the R2 plane.
   */
  Region WhichRegion(const double J1, const double J2) const;

  /**
   * @brief Project point onto feasible set.
   * @params J1  Major axis ratio Ixx/Izz.
   * @params J2  Intermediate axis ratio Iyy/Izz.
   * @return Valid projected inertia ratios point.
   */
  InertiaRatios Project(const double J1, const double J2) const;

  /**
   * @brief Compare two InertiaRatios structs.
   */
  inline friend bool operator==(const InertiaRatios &lhs,
                                const InertiaRatios &rhs) {
    return ((std::abs(lhs.J1 - rhs.J1) < lhs.epsilon) &&
            (std::abs(lhs.J2 - rhs.J2) < lhs.epsilon));
  }

  inline friend std::ostream &operator<<(std::ostream &os,
                                         const InertiaRatios &ir) {
    os << "Inertia ratios: (J1: " << ir.J1 << ", J2: " << ir.J2 << ")"
       << std::endl;
    return os;
  }
};

}  // namespace sph

namespace gtsam {

template <>
struct traits<sph::InertiaRatios> {
  // enum dimension.
  enum { dimension = 2 };
  static int GetDimension(const sph::InertiaRatios &) { return dimension; }

  // Typedefs needed.
  typedef sph::InertiaRatios ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;

  // Local coordinate of InertiaRatios is naive, since we're still in R2.
  static TangentVector Local(const sph::InertiaRatios &origin,
                             const sph::InertiaRatios &other) {
    return TangentVector(other.J1 - origin.J1, other.J2 - origin.J2);
  }

  // Retraction back to InertiaRatios "manifold" by projecting to feasible set.
  static sph::InertiaRatios Retract(const sph::InertiaRatios &origin,
                                    const TangentVector &v) {
    return origin.Project(origin.J1 + v(0), origin.J2 + v(1));
  }

  // Equality with optional tol.
  static bool Equals(const sph::InertiaRatios &p1, const sph::InertiaRatios &p2,
                     const double tol = 1e-8) {
    return ((std::abs(p1.J1 - p2.J1) < tol) && (std::abs(p1.J2 - p2.J2) < tol));
  }

  // Print for testable.
  static void Print(const sph::InertiaRatios &p, const std::string &str = "") {
    std::cout << str << std::endl;
    std::cout << p << std::endl;
  }
};

}  // namespace gtsam

#endif  // SPH_VERTIGO_INERTIARATIOS_H_
