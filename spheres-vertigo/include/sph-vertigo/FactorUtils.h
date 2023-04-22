/**
 * @file FactorUtils.h
 * @brief Utility functions for factor graphs.
 * @date August 05, 2020
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_FACTORUTILS_H_
#define SPH_VERTIGO_FACTORUTILS_H_

#include <gtsam/navigation/CombinedImuFactor.h>

namespace sph {

/**
 * @brief Tune IMU preintegration parameters for SPHERES characteristics.
 * @return Preintegration parameters for GTSAM's combined IMU factors.
 */
boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
GetSpheresPreintegrationParams();

}  // namespace sph

#endif  // SPH_VERTIGO_FACTORUTILS_H_
