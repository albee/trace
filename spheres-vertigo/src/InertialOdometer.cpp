/**
 * @file InertialOdometer.cpp
 * @brief Class for dealing with inertial measurements.
 * @date June 02, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 *
 * Based on code from Tim Setterfield.
 */

#include "sph-vertigo/InertialOdometer.h"

namespace sph {

/* ************************************************************************** */
InertialOdometer::InertialOdometer(const Params &params) : params_(params) {
  // Make sure the mixer matrix has been assigned.
  if (params_.mixer.isApprox(Eigen::MatrixXd::Zero(3, 12)), 1e-10) {
    // clang-format off
    params_.mixer << 1,  1,  0,  0,  0,  0, -1, -1,  0,  0,  0,  0,
                     0,  0,  1,  1,  0,  0,  0,  0, -1, -1,  0,  0,
                     0,  0,  0,  0,  1,  1,  0,  0,  0,  0, -1, -1;
    // clang-format on
  }
}

/* ************************************************************************** */
InertialOdometer::~InertialOdometer() {}

/* ************************************************************************** */
Eigen::Vector3d InertialOdometer::CreateModeledAcceleration(
    const MsgImuData &imu_data, const MsgThrusterTimes &thruster_data) const {
  // The modeled acceleration
  Eigen::Vector3d accel_modeled;

  // Get the start of the time interval, from the IMU's SPHERES time (middle of
  // the interval).
  double t_start =
      static_cast<double>(imu_data.sph_test_time) / 1000.0 - params_.dt / 2;

  // Total number of SPHERES thrusters.
  size_t num_thrusters = 12;

  // Integration timestep for modeled acceleration [s].
  double d_tau = 0.001;
  // Number of integration timesteps in the interval.
  size_t num_int_steps = (size_t)round(params_.dt / d_tau);

  // Vectors of values of 0 or 1 indicating whether the thruster was off or on
  // that millisecond.
  Eigen::MatrixXd thrusters_on =
      Eigen::MatrixXd::Zero(num_thrusters, num_int_steps);

  // Loop through all thrusters for all timesteps and determine whether they
  // were on or off.
  for (size_t j = 0; j < num_thrusters; j++) {
    double t_on = (static_cast<double>(thruster_data.sph_test_time) +
                   static_cast<double>(thruster_data.on_time[j])) /
                      1000.0 +
                  params_.solenoid_delay;
    double t_off = (static_cast<double>(thruster_data.sph_test_time) +
                    static_cast<double>(thruster_data.off_time[j])) /
                       1000.0 +
                   params_.solenoid_delay;

    for (size_t i = 0; i < num_int_steps; i++) {
      double t_curr = t_start + i * d_tau;

      if (t_curr >= t_on && t_curr < t_off) {
        thrusters_on(j, i) = 1.0;
      }
    }
  }

  // Loop through all timesteps and integrate the force (body coords) over the
  // imu period.
  Eigen::Vector3d integrated_force = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < num_int_steps; i++) {
    Eigen::VectorXd thr_on = thrusters_on.block(0, i, num_thrusters, 1);
    Eigen::VectorXd FthNorm =
        pow(params_.thrust_efficiency, thr_on.sum() - 1.0) * thr_on;
    Eigen::Vector3d F_B = params_.thrust_force * params_.mixer * FthNorm;
    integrated_force += F_B * d_tau;
  }

  // Get the average acceleration over this interval
  accel_modeled = (integrated_force / params_.dt) / params_.mass;
  return accel_modeled;
}

}  // namespace sph
