/**
 * @file RigidBodyRotation.h
 * @brief Object to simulate rigid body dynamics.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_RIGIDBODYROTATION_H_
#define SPH_VERTIGO_RIGIDBODYROTATION_H_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {
/**
 * @brief Possible intialization for dynamics.
 *
 * OMEGA0: using \omega_0, ENERGY: \E_k \h_{t_0}.
 */
enum class DynamicsInitType { OMEGA0, ENERGY };

/**
 * @brief Structure to hold the parameters for creating a rigid body object.
 */
struct RigidBodyParams {
  Eigen::Matrix3d J =
      Eigen::Vector3d{3.0, 2.0, 1.0}.asDiagonal();      ///< Inertia ratios.
  Eigen::Matrix3d R_WB0 = Eigen::Matrix3d::Identity();  ///< Initial rot.
  DynamicsInitType init = DynamicsInitType::OMEGA0;     ///< init type.
  // InertiaSymmetry symmetry = InertiaSymmetry::TA;       ///< Symmetry type.

  // Possible initialization parameters.
  Eigen::Vector3d omega0 = Eigen::Vector3d{1.0, 1.0, 1.0};  ///< [rad/s].
  double Ek = 1.0;  ///< Normalized kinetic energy [rad^2/s^2].
  double h = 1.0;   ///< Normalized angular momentum [rad/s].
  double T0 = 0.0;  ///< Initial time [s].
};

/**
 * @brief Class for simulating rigid body dynamics.
 *
 * TODO(tonioteran) Need to make sure that the elliptic integral of the first
 * kind is being correctly computed with respect to the matlab `ellipticF`,
 * since the arguments are different. Boost uses the elliptic modulus `k`, while
 * matlab uses the "parameter" `m = k^2`.
 */
class RigidBodyRotation {
 public:
  /**
   * @brief Constructor.
   */
  explicit RigidBodyRotation(const RigidBodyParams& params);

  // Simple getters.
  inline double J1() const { return p_.J(0, 0); }
  inline double J2() const { return p_.J(1, 1); }
  inline double J3() const { return p_.J(2, 2); }
  inline double Ek() const { return Ek_; }
  inline double h() const { return h_; }
  inline double k() const { return k_; }
  inline double T() const { return T_; }
  inline double T0() const { return T0_; }
  inline double omegaP() const { return omegaP_; }
  inline Eigen::Matrix3d R_WH() const { return R_WH_; }
  inline Eigen::Matrix3d R_WB0() const { return p_.R_WB0; }
  inline Eigen::Vector3i s() const { return s_; }
  inline Eigen::Vector3d omegaMax() const { return omegaMax_; }
  inline Eigen::Vector3d omegaB0_B() const { return omegaB0_0_; }
  inline EnergyState Energy() const { return energy_; }
  inline InertiaSymmetry Symmetry() const { return symmetry_; }

  /**
   * @brief Predict the body angular velocities at time t.
   * @param[in] t  Time of prediction.
   */
  Eigen::Vector3d PredictOmega(const double t) const;

  /**
   * @brief Predict the body angular velocities at multiple times.
   * @param[in] times  Vector of N times at which to predict.
   * @return A 3xN matrix with all predictions horizontally stacked.
   */
  Eigen::MatrixXd PredictOmega(const Eigen::VectorXd& times) const;

  /**
   * @brief Predict the orientation of the ridig body at time t.
   * @param[in] t  Time of prediction.
   * @return The rotation matrix R_WB(t).
   */
  Eigen::Matrix3d PredictOrientation(const double t) const;

  /**
   * @brief Predict the orientation of the ridig body at times t.
   * @param[in] times  Times of prediction.
   * @return Vector with rotation matrices R_WB(t).
   */
  std::vector<Eigen::Matrix3d> PredictOrientation(
      const Eigen::VectorXd& times) const;

  /**
   * @brief Compute the energy state of the object.
   * @param[in] h   Angular momentum magnitude [rad/s].
   * @param[in] Ek  Rotational kinetic energy magnitude [rad^2/s^2].
   * @param[in] J   Inertia ratios matrix [-].
   */
  EnergyState ComputeEnergyState(const double h, const double Ek,
                                 const Eigen::Matrix3d& J) const;

  /**
   * @brief Determine whether inertias are tri-axial, axis-symmetric, or FS.
   * @param[in] h    Angular momentum magnitude [rad/s].
   * @param[in] Ek   Rotational kinetic energy magnitude [rad^2/s^2].
   * @param[in] J    Inertia ratios matrix [-].
   * @param[in] eps  Tolerance in inertia ratios [-].
   */
  InertiaSymmetry ComputeInertiaSymmetry(const double h, const double Ek,
                                         const Eigen::Matrix3d& J,
                                         const double eps = 1e-4) const;

  /**
   * @brief Get the signs by convention.
   * @param[in] symmetry  Type of inertia symmetry for the rigid body.
   * @param[in] energy    Current energy state for the rigid body.
   *
   * Not currently taking into account Medium Energy for Tri-axial spins.
   */
  Eigen::Vector3i GetOmegaConventionSigns(const InertiaSymmetry& symmetry,
                                          const EnergyState& energy) const;

  /**
   * @brief Get maximum angular velocities.
   * @param[in] h    Angular momentum magnitude [rad/s].
   * @param[in] Ek   Rotational kinetic energy magnitude [rad^2/s^2].
   * @param[in] J    Inertia ratios matrix [-].
   * @param[in] symmetry  Type of inertia symmetry for the rigid body.
   * @param[in] energy    Current energy state for the rigid body.
   */
  Eigen::Vector3d GetMaxOmegas(const double h, const double Ek,
                               const Eigen::Matrix3d& J,
                               const InertiaSymmetry& symmetry,
                               const EnergyState& energy) const;

  /**
   * @brief Get the parameter of the Jacobi elliptic functions.
   * @param[in] h    Angular momentum magnitude [rad/s].
   * @param[in] Ek   Rotational kinetic energy magnitude [rad^2/s^2].
   * @param[in] J    Inertia ratios matrix [-].
   * @param[in] energy    Current energy state for the rigid body.
   */
  double GetEllipticModulus(const double h, const double Ek,
                            const Eigen::Matrix3d& J,
                            const EnergyState& energy) const;

  /**
   * @brief Get the body nutation rate.
   * @param[in] h    Angular momentum magnitude [rad/s].
   * @param[in] Ek   Rotational kinetic energy magnitude [rad^2/s^2].
   * @param[in] J    Inertia ratios matrix [-].
   * @param[in] symmetry  Type of inertia symmetry for the rigid body.
   * @param[in] energy    Current energy state for the rigid body.
   */
  double ComputeBodyNutationRate(const double h, const double Ek,
                                 const Eigen::Matrix3d& J,
                                 const InertiaSymmetry& symmetry,
                                 const EnergyState& energy) const;

  /**
   * @brief Calculate the quarter-period of sn and cn, half period of dn [s].
   * @param[in] symmetry  Type of inertia symmetry for the rigid body.
   * @param[in] k         The modulus for the elliptic function.
   * @param[in] omegaP    The body nutation rate.
   * @return Quarter period T [s].
   *
   * sn and cn are periodic every 4T secs, and dn is periodic every 2T secs.
   */
  double CalculateT(const InertiaSymmetry& symmetry, const double k,
                    const double omegaP) const;

  /**
   * @brief Calculate the inital time if not previously specified.
   * @param[in] symmetry   Type of inertia symmetry for the rigid body.
   * @param[in] energy     Current energy state for the rigid body.
   * @param[in] s          Signs of angular velocities.
   * @param[in] omegaB0_B  Initial angular velocity.
   * @param[in] omegaMax   Maximum angular velocities.
   * @param[in] omegaP     Body nutation rate.
   * @param[in] T          Quarter period of sn and cn.
   * @param[in] k          Elliptic modulus.
   * @return The initial time for the experiment [s].
   */
  double CalculateT0(const InertiaSymmetry& symmetry, const EnergyState& energy,
                     const Eigen::Vector3i& s, const Eigen::Vector3d& omegaB0_B,
                     const Eigen::Vector3d& omegaMax, const double omegaP,
                     const double T, const double k) const;

  /**
   * @brief Detect principal axis rotations, find rotation axis and init omega.
   *
   * TODO(tonioteran) Implement.
   */
  void DetectPrincipalAxesRotation() const;

  /**
   * @brief Determine rotation axis (0's for multi-axis) and omega0 if reqd.
   * @param[in]  symmetry   Type of inertia symmetry for the rigid body.
   * @param[in]  energy     Current energy state for the rigid body.
   * @param[out] omegaB0_B  Initial value for angular momentum.
   * @return Rotation axis.
   *
   * Initial angular velocity calculated only if initialized using energy and h.
   */
  Eigen::Vector3d DetermineRotationAxis(const InertiaSymmetry& symmetry,
                                        const EnergyState& energy,
                                        const DynamicsInitType& init,
                                        Eigen::Vector3d* omegaB0_B) const;

  /**
   * @brief Calculate the rotation matrix R_BH(t) at time t.
   * @param[in]  t  Time at which to calculate the orientation.
   * @return The rotation matrix R_BH(t).
   */
  Eigen::Matrix3d CalculateRotBH(const double t) const;

  /**
   * @brief Get orientation of angular momentum frame wrt world frame.
   * @return The rotation matrix R_WH.
   */
  Eigen::Matrix3d CalculateRotWH() const;

  /**
   * @brief Print out all the motion model details
   */
  void Print() const;

 private:
  /**
   * @brief Update the internal parameters.
   */
  void Update();

  RigidBodyParams p_;          ///< Internal copy of initial paramaters.
  Eigen::Matrix3d R_WH_;       ///< Orientation of ang momentum frame wrt W.
  Eigen::Vector3i s_;          ///< Signs of angular velocities.
  EnergyState energy_;         ///< Energy state of rigid body.
  InertiaSymmetry symmetry_;   ///< Tri-axial, axis-symmetric?
  Eigen::Vector3d omegaMax_;   ///< Maximum angular velocities.
  Eigen::Vector3d omegaB0_0_;  ///< Initial body angular velocity.
  Eigen::Vector3d rotAxis_;    ///< Rotation axis.

  double k_;       ///< Modulus for the Jacobi elliptic function.
  double T_;       ///< Quarter-period of sn and cn, half-period of dn [s].
  double T0_;      ///< Initial time when sn(t0) = 0, cn(t0) = 1, dn(t0)=1 [s].
  double h_;       ///< Normalized angular momentum [rad/s].
  double Ek_;      ///< Normalized kinetic energy [rad^2/s^2].
  double omegaP_;  ///< Body nutation rate.
};

}  // namespace sph

#endif  // SPH_VERTIGO_RIGIDBODYROTATION_H_
