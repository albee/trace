/**
 * @file    InertiaRatiosManifoldFactor.h
 * @brief   On-manifold estimation of inertia ratios using cons of ang momentum.
 * @author  Tonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <string>

#include "sph-vertigo/InertiaRatios.h"

namespace gtsam {

/**
 * @brief A factor for enforcing the conservation of angular momentum.
 *
 * NOTE(tonioteran)
 */
class InertiaRatiosManifoldFactor
    : public NoiseModelFactor6<sph::InertiaRatios, Vector3, Pose3, Pose3, Pose3,
                               Pose3> {
 private:
  typedef NoiseModelFactor6<sph::InertiaRatios, Vector3, Pose3, Pose3, Pose3,
                            Pose3>
      Base;

  Rot3 R_BG_;  /// Rotation from moving frame G to target's principal axes B.
  double deltaT_ij_;  /// Time difference between step i and j.

 public:
  /// Default constructor, only for serialization.
  InertiaRatiosManifoldFactor() {}

  /**
   * @brief Constructor. Note: factor creates a measurement by conditioning.
   * @param J       Inertia ratios vector with J1 and J2.
   * @param h_ij    Angular momentum vector h_ij = J * omega_ij.
   * @param T_GiBi  Observer body pose relative to moving G frame at time i.
   * @param T_GjBj  Observer body pose relative to moving G frame at time j.
   * @param T_WBi   Observer body pose relative to stationary W frame at time i.
   * @param T_WBj   Observer body pose relative to stationary W frame at time j.
   * @param model   Gaussian noise model for the angular velocity "measurement".
   * @param R_BG    Rotation from moving frame G to target principal axes B.
   * @param deltaT_ij   Time difference between step i and j.
   */
  InertiaRatiosManifoldFactor(const Key& J, const Key& h_ij, const Key& T_GiBi,
                              const Key& T_GjBj, const Key& T_WBi,
                              const Key& T_WBj, const SharedNoiseModel& model,
                              const Rot3& R_BG, const double deltaT_ij);

  /// Destructor.
  ~InertiaRatiosManifoldFactor() {}

  /**
   * @brief Error, where the nominal measurement is the zero vector [0 0 0]'.
   * @param Values  for the 6 variables involved in the factor.
   * @param H1      Jacobian of the error wrt the inertia ratios J.
   * @param H2      Jacobian of the error wrt the angular momentum h_ij.
   * @param H3toH6  Jacobian of the error wrt all four poses (3x6 zero blocks).
   */
  Vector evaluateError(const sph::InertiaRatios& J, const Vector3& h_ij,
                       const Pose3& T_GiBi, const Pose3& T_GjBj,
                       const Pose3& T_WBi, const Pose3& T_WBj,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none,
                       boost::optional<Matrix&> H4 = boost::none,
                       boost::optional<Matrix&> H5 = boost::none,
                       boost::optional<Matrix&> H6 = boost::none) const;

  /** Print. */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /**
   * Check if equal within a tolerance
   * - satisfies requirements of Testable
   * */
  using NonlinearFactor::equals;  // To let compiler know we are not hiding.
  bool equals(const InertiaRatiosManifoldFactor& factor,
              double tol = 1e-9) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
  }
};

// Implement the Testable traits
template <>
struct traits<InertiaRatiosManifoldFactor>
    : public Testable<InertiaRatiosManifoldFactor> {};

}  // end namespace gtsam
