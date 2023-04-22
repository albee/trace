/**
 * @file InertiaRatiosOpt.h
 * @brief Optimization methods for offline inertia ratios.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_INERTIARATIOSOPT_H_
#define SPH_VERTIGO_INERTIARATIOSOPT_H_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @brief Parameters for instantiating an inertia ratios optimizer object.
 */
struct InertiaRatiosOptParams {
  Eigen::MatrixXd omegaB_B_;   ///< (3 x nT) matrix of rigid body angular
                               ///< velocity data in the body frame.
  Eigen::MatrixXd covOmegaB_;  ///< (3 x 3nT) matrix of angular velocity
                               ///< covariances in the body frame.
  Eigen::Matrix3d R_WB_;       ///< (3x3) matrix of rigid body orientation.
  std::vector<double> t_;      ///< Vector of times when omegaB_B_ was sampled.
  EnergyState energyState_;    ///< Energy state of the rigid body.
  InertiaSymmetry symmetry_;   ///< Type of mass distribution of rigid body.
  Eigen::Vector3d omegaMax_;   ///< Max velocity fit in each direction.
};

class InertiaConstraints {
  InertiaConstraints();
};

/**
 * @brief Class based on methods described by [1].
 *
 * [1] Setterfield, Timothy P., et al. "Inertial Properties Estimation of a
 * Passive On-orbit Object Using Polhode Analysis." JGCD (2018): 2214-2231.
 */
class InertiaRatiosOpt {
 public:
  /**
   * @brief Constructor
   */
  explicit InertiaRatiosOpt(const InertiaRatiosOptParams& params);

 private:
  Eigen::Vector3d p0_;
  std::shared_ptr<InertiaConstraints> constraints_;  ///< Constraints object.
};

}  // namespace sph

#endif  // SPH_VERTIGO_INERTIARATIOSOPT_H_
