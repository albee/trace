/**
 * @file InertialOdometer.h
 * @brief Class for dealing with inertial measurements.
 * @date June 02, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 *
 * Based on code from Tim Setterfield.
 */

#ifndef SPH_VERTIGO_INERTIALODOMETER_H_
#define SPH_VERTIGO_INERTIALODOMETER_H_

#include <Eigen/Dense>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @class InertialOdometer
 * @brief Handle SPHERES related inertial measurements.
 *
 * Makes use of the SPHERES physical parameter, thruster configuration, and
 * mixer matrix to model accelerations based on thruster firing times. Takes
 * into account the thruster efficiency and reduces thrust force by
 * `thrust_efficiency_^(n_thr - 1)` when more than one thruster fires.
 */
class InertialOdometer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Structure to hold the inertial odometer's parameters.
  struct Params {
    // 50 Hz default.
    double dt = 0.02;  ///< Nominal time between IMU measurements [s].

    // From Chris Jewison's master thesis.
    double mass = 4.43 + 1.745;   ///< SPHERES + Goggles assembly [kg].
    double thrust_force = 0.098;  ///< Thrust force of a single thruster [N].
    double thrust_efficiency = 0.94;  ///< Thruster efficienty [-].

    // From SPHERES GSP document.
    double solenoid_delay = 0.006;  ///< Commanded v. actual open delay [s].

    Eigen::Matrix<double, 3, 12> mixer;  ///< Force mixer matrix [-].

    /// Continuous-time sqrt(PSD * Hz) from [NoletPhD pp. 270] <=0.05
    /// [(deg/sec)/sqrt(Hz)] for Systron Donner BEI Gyrochip II single-axis rate
    /// gyroscopes. See also [Farrell2008 pp. 124], [rad/s?].
    double sigma_g_ = 0.05 * M_PI / 180.0;
    double sigma_bias_g_ = 0.000001;  ///< Continuos-time stddev in gyroscope
                                      ///< bias random walk, guessed [rad/s/s].

    /// Continuous-time standard deviation in artificial accelerometer
    /// measurements [(m/s^2)/sqrt(Hz)]. Use the acceleration of a single
    /// thruster applied over 1 ms as an RMS estimate over a 0-1 Hz frequency
    /// range as the process noise (** not sure if valid), [m/s^2].
    double sigma_a_ = thrust_force * 0.001 / mass;
    double sigma_bias_a_ =
        0.0000000001;  ///< Continuos-time stddev in accelerometer
                       ///< bias random walk, guessed [m/s^2/s].
  };

  /**
   * @brief Default constructor.
   * @param params Configuration parameters for the inertial odometer.
   */
  explicit InertialOdometer(const Params &params);

  /**
   * @brief Default destructor.
   */
  ~InertialOdometer();

  /**
   * @brief Getter for the inertial odometer's initialization parameters.
   * @return Parameters struct used for construction.
   */
  Params Parameters() const { return params_; }

  /**
   * @brief Uses firing times and physical params to model accel measurement.
   * @param imu_data Scaled and unbiased SPHERES IMU message.
   * @param thruster_data SPHERES on/off thruster times.
   * @return Artificially generated acceleration over that interval.
   */
  Eigen::Vector3d CreateModeledAcceleration(
      const MsgImuData &imu_data, const MsgThrusterTimes &thruster_data) const;

 private:
  Params params_;  ///< Internal copy of the object's initalization values.
};

}  // namespace sph

#endif  // SPH_VERTIGO_INERTIALODOMETER_H_
