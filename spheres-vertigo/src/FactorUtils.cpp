/**
 * @file FactorUtils.h
 * @brief Utility functions for factor graphs.
 * @date August 05, 2020
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/FactorUtils.h"

namespace sph {

/* ************************************************************************** */
boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
GetSpheresPreintegrationParams() {
  // From Chris Jewison's master thesis.
  double mass = 4.43 + 1.745;  // SPHERES + Goggles assembly [kg].
  double thrustForce = 0.098;  // Thrust force of a single thruster [N].

  // Continuous-time sqrt(PSD * Hz) from [NoletPhD pp. 270] <=0.05
  // [(deg/sec)/sqrt(Hz)] for Systron Donner BEI Gyrochip II single-axis rate
  // gyroscopes. See also [Farrell2008 pp. 124]
  double sigmaGyro = 0.05 * M_PI / 180.0;

  // Continuous-time standard deviation in artificial accelerometer measurements
  // [(m/s^2)/sqrt(Hz)]. Use the acceleration of a single thruster applied over
  // 1 ms as an RMS estimate over a 0-1 Hz frequency range as the process noise
  // (** not sure if valid)
  double sigmaAccel = thrustForce * 0.001 / mass;

  // Continuous-time standard deviation in gyroscope bias random walk [rad/s/s]
  // This is just a guessed value
  // NOTE: Does this matter for constant biases?
  double sigmaBiasGyro = 0.000001;

  // Continuous-time standard deviation in accelerometer bias random walk
  // [m/s^2/s] This is just a guessed value, made to be very small since there
  // should be no bias in artificially generated accelerometer measurements
  // NOTE: Does this matter for constant biases?
  double sigmaBiasAccel = 0.0000000001;

  // Initialize preintegration parameters, with zero-g, gyro noise, and
  // accelerometer noise
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
      preintegrationParams(new gtsam::PreintegratedCombinedMeasurements::Params(
          gtsam::Vector::Zero(3)));

  // Zero gravity
  preintegrationParams->n_gravity = gtsam::Vector::Zero(3);

  // In orbit, the SPHERES navigation frame will rotate about the -y axis at a
  // rate of: (360 deg/orbit) / (92.69 mins/orbit) / (60 mins/sec) = 0.06473
  // deg/s = 0.0011298 rad/s NOTE: In addition to correcting the gyro
  // measurements for this value, GTSAM also uses this to correct for coriolis
  // on acceleration, which affect the velocity and position integration. With
  // our acceleration pseudo-measurements, these terms are not present, and this
  // correction is unnecessary.
  // NOTE: Create a custom version of the coriolis() function in NavState.cpp
  // line:288 that does not perform corrections for position (dP) and velocity
  // (dV).
  preintegrationParams->omegaCoriolis =
      (gtsam::Vector(3) << 0.0, -0.0011298, 0.0).finished();

  // Do not use second order coriolis, which only corrects position and velocity
  // integration
  preintegrationParams->use2ndOrderCoriolis = false;

  //    preintegrationParams->body_P_sensor = PBtoIfict;
  preintegrationParams->gyroscopeCovariance =
      sigmaGyro * sigmaGyro * gtsam::Matrix3::Identity();
  preintegrationParams->accelerometerCovariance =
      sigmaAccel * sigmaAccel * gtsam::Matrix3::Identity();
  preintegrationParams->biasOmegaCovariance =
      sigmaBiasGyro * sigmaBiasGyro * gtsam::Matrix3::Identity();
  preintegrationParams->biasAccCovariance =
      sigmaBiasAccel * sigmaBiasAccel * gtsam::Matrix3::Identity();

  // NOTE: I am unsure what these are for -- set to identity for now
  // --- need to be lower than identity or inertial factors are considered very
  // inaccurate
  preintegrationParams->biasAccOmegaInt = 0.00001 * gtsam::Matrix6::Identity();
  preintegrationParams->integrationCovariance =
      0.0001 * gtsam::Matrix3::Identity();

  return preintegrationParams;
}

}  // namespace sph
