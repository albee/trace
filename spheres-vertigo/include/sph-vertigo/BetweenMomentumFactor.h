/**
 * @file    BetweenMomentumFactor.h
 * @brief   Random walk factor to constrain angular momentum.
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

namespace gtsam {

/**
 * @brief A factor for enforcing the conservation of angular momentum.
 *
 * NOTE(tonioteran)
 */
class BetweenMomentumFactor
    : public NoiseModelFactor4<Vector3, Vector3, Vector3, Vector3> {
 private:
  typedef NoiseModelFactor4<Vector3, Vector3, Vector3, Vector3> Base;

 public:
  /// Default constructor, only for serialization.
  BetweenMomentumFactor() {}

  /**
   * @brief Constructor. Note: no measurement, just random walk model.
   * @param h_i    Angular momentum vector h_i = J * omega_i.
   * @param h_j    Angular momentum vector h_j = J * omega_j.
   * @param bh_i   Bias to allow for external torques at time i.
   * @param bh_j   Bias to allow for external torques at time j.
   * @param model  Gaussian noise model for the random walk magnitude.
   */
  BetweenMomentumFactor(const Key& h_i, const Key& h_j, const Key& bh_i,
                        const Key& bh_j, const SharedNoiseModel& model);

  /// Destructor.
  ~BetweenMomentumFactor() {}

  /**
   * @brief Error, where the conservation of angular momentum is enforced.
   * @param h_i    Angular momentum vector h_i = J * omega_i.
   * @param h_j    Angular momentum vector h_j = J * omega_j.
   * @param bh_i   Bias to allow for external torques at time i.
   * @param bh_j   Bias to allow for external torques at time j.
   * @param H1     Jacobian of the error wrt the angular momentum h_i.
   * @param H2     Jacobian of the error wrt the angular momentum h_j.
   * @param H3     Jacobian of the error wrt the bias at time i.
   * @param H4     Jacobian of the error wrt the bias at time j.
   */
  Vector evaluateError(const Vector3& h_i, const Vector3& h_j,
                       const Vector3& bh_i, const Vector3& bh_j,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none,
                       boost::optional<Matrix&> H4 = boost::none) const;

  /** Print. */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /**
   * Check if equal within a tolerance
   * - satisfies requirements of Testable
   * */
  using NonlinearFactor::equals;  // To let compiler know we are not hiding.
  bool equals(const BetweenMomentumFactor& factor, double tol = 1e-9) const;

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
struct traits<BetweenMomentumFactor> : public Testable<BetweenMomentumFactor> {
};

}  // end namespace gtsam
