/**
 * @file RigidBodyRotation.h
 * @brief Object to simulate rigid body dynamics.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/RigidBodyRotation.h"

#include <boost/math/special_functions/ellint_1.hpp>
#include <boost/math/special_functions/ellint_3.hpp>
#include <boost/math/special_functions/jacobi_elliptic.hpp>

namespace sph {

/* ************************************************************************** */
RigidBodyRotation::RigidBodyRotation(const RigidBodyParams& params)
    : p_(params) {
  Update();
}

/* ************************************************************************** */
void RigidBodyRotation::Update() {
  // Calculate kinetic energy and ang mom if initialized w/ ang velocity.
  if (p_.init == DynamicsInitType::OMEGA0) {
    Ek_ = 0.5 * p_.omega0.transpose() * p_.J * p_.omega0;
    h_ = (p_.J * p_.omega0).norm();
    omegaB0_0_ = p_.omega0;
  } else {
    Ek_ = p_.Ek;
    h_ = p_.h;
    T0_ = p_.T0;
  }

  energy_ = ComputeEnergyState(h_, Ek_, p_.J);
  symmetry_ = ComputeInertiaSymmetry(h_, Ek_, p_.J);
  s_ = GetOmegaConventionSigns(symmetry_, energy_);
  omegaMax_ = GetMaxOmegas(h_, Ek_, p_.J, symmetry_, energy_);
  k_ = GetEllipticModulus(h_, Ek_, p_.J, energy_);
  omegaP_ = ComputeBodyNutationRate(h_, Ek_, p_.J, symmetry_, energy_);
  T_ = CalculateT(symmetry_, k_, omegaP_);

  if (p_.init == DynamicsInitType::OMEGA0) {
    T0_ = CalculateT0(symmetry_, energy_, s_, omegaB0_0_, omegaMax_, omegaP_,
                      T_, k_);
  }

  // Calculate the orientation of angular momentum frame H wrt W.
  // R_WH_ = p_.R_WB0 * CalculateRotWH();
  R_WH_ = CalculateRotWH();

  // Print();
}

/* ************************************************************************** */
EnergyState RigidBodyRotation::ComputeEnergyState(
    const double h, const double Ek, const Eigen::Matrix3d& J) const {
  double q = (h * h) - (2 * Ek * J(1, 1));  // Using J2.
  if (q > 1e-10) {
    return EnergyState::LE;
  } else if (std::abs(q) <= 1e-10) {
    return EnergyState::ME;
  } else {
    return EnergyState::HE;
  }
}

/* ************************************************************************** */
InertiaSymmetry RigidBodyRotation::ComputeInertiaSymmetry(
    const double h, const double Ek, const Eigen::Matrix3d& J,
    const double eps) const {
  double J1 = J(0, 0), J2 = J(1, 1), J3 = J(2, 2);

  if ((J1 >= (J2 + eps)) && (J2 >= (J3 + eps))) {
    return InertiaSymmetry::TA;
  } else if ((J1 >= (J2 + eps)) && (J1 >= (J3 + eps)) && ((J2 - J3) <= eps)) {
    // double Ja = J1, Jt = J3;  // Axial and transverse inertias.
    return InertiaSymmetry::AS1;
  } else if ((J1 >= (J3 + eps)) && (J2 >= (J3 + eps)) && ((J2 - J1) <= eps)) {
    // double Ja = J3, Jt = J1;  // Axial and transverse inertias.
    return InertiaSymmetry::AS3;
  } else {
    return InertiaSymmetry::FS;
  }
}

/* ************************************************************************** */
Eigen::Vector3i RigidBodyRotation::GetOmegaConventionSigns(
    const InertiaSymmetry& symmetry, const EnergyState& energy) const {
  if (symmetry == InertiaSymmetry::TA) {
    if (energy == EnergyState::LE) {
      return Eigen::Vector3i{1, -1, 1};
    } else {
      return Eigen::Vector3i{-1, 1, 1};
    }
  } else {
    return Eigen::Vector3i{1, 1, 1};
  }
}

/* ************************************************************************** */
Eigen::Vector3d RigidBodyRotation::GetMaxOmegas(
    const double h, const double Ek, const Eigen::Matrix3d& J,
    const InertiaSymmetry& symmetry, const EnergyState& energy) const {
  Eigen::Vector3d omegaMax{0.0, 0.0, 0.0};
  double J1 = J(0, 0), J2 = J(1, 1), J3 = J(2, 2);

  if (symmetry == InertiaSymmetry::TA) {
    omegaMax(0) = std::sqrt((h * h - 2 * Ek * J3) / (J1 * (J1 - J3)));
    if ((energy == EnergyState::LE) || (energy == EnergyState::ME)) {
      omegaMax(1) = std::sqrt((2 * Ek * J1 - h * h) / (J2 * (J1 - J2)));
    } else if (energy == EnergyState::HE) {
      omegaMax(1) = std::sqrt((h * h - 2 * Ek * J3) / (J2 * (J2 - J3)));
    }
    omegaMax(2) = std::sqrt((2 * Ek * J1 - h * h) / (J3 * (J1 - J3)));
  }

  return omegaMax;
}

/* ************************************************************************** */
double RigidBodyRotation::GetEllipticModulus(const double h, const double Ek,
                                             const Eigen::Matrix3d& J,
                                             const EnergyState& energy) const {
  double J1 = J(0, 0), J2 = J(1, 1), J3 = J(2, 2);
  double a = std::sqrt((h * h - 2 * Ek * J3) / (J2 * (J2 - J3)));
  double b = std::sqrt((2 * Ek * J1 - h * h) / (J2 * (J1 - J2)));

  double k;
  if (energy == EnergyState::LE) {
    k = b / a;
  } else if (energy == EnergyState::ME) {
    k = 1;
  } else {  // EnergyState::HE
    k = a / b;
  }
  k = std::max(0.0, std::min(1.0, k));

  return k;
}

/* ************************************************************************** */
double RigidBodyRotation::ComputeBodyNutationRate(
    const double h, const double Ek, const Eigen::Matrix3d& J,
    const InertiaSymmetry& symmetry, const EnergyState& energy) const {
  double J1 = J(0, 0), J2 = J(1, 1), J3 = J(2, 2);
  double omegaP = 0;

  if (symmetry == InertiaSymmetry::TA) {
    if (energy == EnergyState::LE || energy == EnergyState::ME) {
      omegaP = std::sqrt((h * h - 2 * Ek * J3) * (J1 - J2) / (J1 * J2 * J3));
    } else {
      omegaP = std::sqrt((2 * Ek * J1 - h * h) * (J2 - J3) / (J1 * J2 * J3));
    }
  } else if (symmetry == InertiaSymmetry::AS1) {
    // double Ja = J1, Jt = J3;  // Axial and transverse inertias.
    // omegaP = -1;              // TODO(tonioteran) NOT IMPLEMENTED.
    std::cout << "RigidBodyRotation::ComputeBodyNutationRate" << std::endl;
    std::cout << "InertiaSymmetry::AS1 not implemented." << std::endl;
    std::exit(1);
  } else if (symmetry == InertiaSymmetry::AS3) {
    std::cout << "RigidBodyRotation::ComputeBodyNutationRate" << std::endl;
    std::cout << "InertiaSymmetry::AS3 not implemented." << std::endl;
    // double Ja = J3, Jt = J1;  // Axial and transverse inertias.
    // omegaP = -1;              // TODO(tonioteran) NOT IMPLEMENTED.
  }

  return omegaP;
}

/* ************************************************************************** */
double RigidBodyRotation::CalculateT(const InertiaSymmetry& symmetry,
                                     const double k,
                                     const double omegaP) const {
  double T = 0;
  if (symmetry == InertiaSymmetry::TA) {
    // Get the parameter using the modulus.
    // double m = k * k;
    // Get the period in terms of u (e.g., sn(u, m)).
    double Tu = boost::math::ellint_1(k, M_PI / 2);  // Args: (k, phi).
    // Get the period in terms of time t (e.g., sn(omegaP*t, m)) [s].
    T = Tu / omegaP;
  } else if (symmetry == InertiaSymmetry::AS1 ||
             symmetry == InertiaSymmetry::AS3) {
    // Get the period as the inverse of frequency.
    T = 2 * M_PI * std::abs(1 / omegaP);
  }

  return T;
}

/* ************************************************************************** */
double RigidBodyRotation::CalculateT0(const InertiaSymmetry& symmetry,
                                      const EnergyState& energy,
                                      const Eigen::Vector3i& s,
                                      const Eigen::Vector3d& omegaB0_B,
                                      const Eigen::Vector3d& omegaMax,
                                      const double omegaP, const double T,
                                      const double k) const {
  double T0 = 0;
  // Put initial omega into canonical form, with domain [-1,1], and signs
  // appropriate to the canonical versions of sn, cn, and dn.
  Eigen::Vector3d omega0_Bc{s(0) * omegaB0_B(0) / omegaMax(0),
                            s(1) * omegaB0_B(1) / omegaMax(1),
                            s(2) * omegaB0_B(2) / omegaMax(2)};
  // Fix numerical issues.
  for (int i = 0; i < 2; i++) {
    if (omega0_Bc(i) > 1) omega0_Bc(i) = 1;
    if (omega0_Bc(i) < -1) omega0_Bc(i) = -1;
  }

  if (symmetry == InertiaSymmetry::TA) {
    // Invert the Jacobi elliptic functions. First determine which angular
    // velocities correspond to the sn and cn curves, respectively.
    double omega0_Bc_sn = omega0_Bc(1);
    double omega0_Bc_cn = 0;
    if (energy == EnergyState::LE) {
      omega0_Bc_cn = omega0_Bc(2);
    } else if (energy == EnergyState::ME) {
      omega0_Bc_cn = omega0_Bc(2);  // Could also use omega0_Bc(0), also sech.
    } else {                        // EnergyState::HE
      omega0_Bc_cn = omega0_Bc(0);
    }

    // Get quarter-period of the elliptic function in terms of u (sn(u,m)).
    double Tu = T * omegaP;

    // Get the value asn, which will be between -Tu and Tu.
    // double m = k * k;  // Parameter from elliptic modulus.
    double u = boost::math::ellint_1(k, std::asin(omega0_Bc_sn));  // (k, phi).

    // Determine in which quadrant the resulting solition is.
    if (omega0_Bc_sn >= 0 && omega0_Bc_cn >= 0) {  // Q1.
      // Stays the same.
    } else if ((omega0_Bc_sn >= 0 && omega0_Bc_cn < 0) ||
               (omega0_Bc_sn < 0 && omega0_Bc_cn < 0)) {  // Q2 or Q3.
      u = 2 * Tu - u;
    } else {  // Q4.
      u = 4 * Tu + u;
    }

    // Magnitude t0 is u/omegaP, but negate so that experiment starts at 0.
    T0 = -u / omegaP;

  } else {
    std::cout << "RigidBodyRotation::CalculateT0" << std::endl;
    std::cout << "InertiaSymmetry::AS{1,3} not implemented." << std::endl;
    std::exit(1);
  }

  return T0;
}

/* ************************************************************************** */
Eigen::Vector3d RigidBodyRotation::DetermineRotationAxis(
    const InertiaSymmetry& symmetry, const EnergyState& energy,
    const DynamicsInitType& init, Eigen::Vector3d* omegaB0_B) const {
  Eigen::Vector3d rotAxis{0.0, 0.0, 0.0};
  if (symmetry == InertiaSymmetry::TA) {
    rotAxis = Eigen::Vector3d{0.0, 0.0, 0.0};
    if (init == DynamicsInitType::ENERGY) {
      *omegaB0_B = PredictOmega(0);
    }
  } else {
    std::cout << "RigidBodyRotation::DetermineRotationAxis" << std::endl;
    std::cout << "InertiaSymmetry::AS{1,3} not implemented." << std::endl;
    std::exit(1);
  }

  return rotAxis;
}

/* ************************************************************************** */
Eigen::Matrix3d RigidBodyRotation::CalculateRotBH(const double t) const {
  Eigen::Matrix3d R1, R2, R_BH;

  // Get the current and maximum angular velocity.
  Eigen::Vector3d omegaB_B = PredictOmega(t);
  double omega1 = omegaB_B(0), omega2 = omegaB_B(1), omega3 = omegaB_B(2);
  double omega1m = omegaMax_(0), omega2m = omegaMax_(1), omega3m = omegaMax_(2);

  // Get inertia tensor, angular momentum, components and their maxes.
  double J1 = p_.J(0, 0), J2 = p_.J(1, 1), J3 = p_.J(2, 2);
  double h = h_;
  double h1 = J1 * omega1, h2 = J2 * omega2, h3 = J3 * omega3;
  double h1m = J1 * omega1m, h2m = J2 * omega2m, h3m = J3 * omega3m;

  if (symmetry_ == InertiaSymmetry::TA) {
    // Determine the number of cycles of phi for finding alpha.
    double numCycles = std::floor((t - T0_) / (4 * T_));

    // Get the elliptic functions for finding alpha.
    double snUm, cnUm;
    snUm = boost::math::jacobi_elliptic(k_, omegaP_ * (t - T0_), &cnUm);

    // Get values of phi at time=t for finding alpha, using quadrants to
    // properly determine phiM (NOTE: asin returns values [-pi/2, pi/2]).
    double phiM = 0;
    if (snUm >= 0 && cnUm >= 0) {  // Q1
      phiM = numCycles * 2 * M_PI + std::asin(snUm);
    } else if ((snUm >= 0 && cnUm < 0) || (snUm < 0 && cnUm < 0)) {  // Q2 or Q3
      phiM = numCycles * 2 * M_PI + M_PI - std::asin(snUm);
    } else {  // Q4
      phiM = numCycles * 2 * M_PI + 2 * M_PI + std::asin(snUm);
    }

    // Determine rotation matrix based on energy level.
    if (energy_ == EnergyState::LE) {
      double A = std::sqrt(1 - std::pow(h1 / h, 2));
      double a0 = (J2 * J2 * J3 - J2 * J3 * J3) / (J1 * J2 - J1 * J3);
      double n = (h2m * h2m - h3m * h3m) / (h3m * h3m);
      double ellint3 = 1 / omegaP_ * boost::math::ellint_3(k_, -n, phiM);
      double alpha = h / (J2 * J3) * (a0 * (t - T0_) + (J2 - a0) * ellint3);

      // clang-format off
      // R1 << h1/h,            h2/h,            h3/h,
      //          A,  -h1*h2/(A*h*h),  -h1*h3/(A*h*h),
      //          0,        h3/(A*h),       -h2/(A*h);
      R1 << h1/h,               A,          0,
            h2/h,  -h1*h2/(A*h*h),   h3/(A*h),
            h3/h,  -h1*h3/(A*h*h),  -h2/(A*h);
      R2 << 1,                0,               0,
            0,  std::cos(alpha), std::sin(alpha),
            0, -std::sin(alpha), std::cos(alpha);
      // clang-format on.

    } else if (energy_ == EnergyState::HE) {
      double A = std::sqrt(1 - std::pow(h3 / h, 2));
      double a0 = (J1 * J1 * J2 - J1 * J2 * J2) / (J1 * J3 - J2 * J3);
      double n = (h2m * h2m - h1m * h1m) / (h1m * h1m);
      double ellint3 = 1 / omegaP_ * boost::math::ellint_3(k_, -n, phiM);
      double alpha = h / (J1 * J2) * (a0 * (t - T0_) + (J2 - a0) * ellint3);

      // clang-format off
      R1 << -h1*h3/(A*h*h),  h2/(A*h), h1/h,
            -h2*h3/(A*h*h), -h1/(A*h), h2/h,
                         A,         0, h3/h;
      R2 << std::cos(alpha), std::sin(alpha), 0,
           -std::sin(alpha), std::cos(alpha), 0,
                          0,               0, 1;
      // clang-format on

    } else {
      std::cout << "RigidBodyRotation::CalculateRotBH" << std::endl;
      std::cout << "EnergyState::ME not implemented." << std::endl;
    }

    R_BH = R1 * R2;

  } else {
    std::cout << "RigidBodyRotation::CalculateRotBH" << std::endl;
    std::cout << "InertiaSymmetry::notTA not implemented." << std::endl;
  }

  return R_BH;
}

/* ************************************************************************** */
Eigen::Matrix3d RigidBodyRotation::CalculateRotWH() const {
  // Calculate as in [Hurtado2014 Sec V].
  return (p_.R_WB0 * CalculateRotBH(0));  // R_WB0 * R_B0H = R_WH
}

/* ************************************************************************** */
void RigidBodyRotation::Print() const {
  std::cout << "RigidBodyRotation: ---" << std::endl
            << "Energy state: " << EnergyString(energy_) << std::endl
            << "Inertia symmetry: " << InertiaString(symmetry_) << std::endl
            << "Rotation axis: " << rotAxis_.transpose() << std::endl
            << "[J1 J2 J3]: " << p_.J(0, 0) << " " << p_.J(1, 1) << " "
            << p_.J(2, 2) << std::endl
            << "omegaP: " << omegaP_ << " [rad/s]" << std::endl
            << "T: " << T_ << " [s]" << std::endl
            << "T0: " << T0_ << " [s]" << std::endl
            << "omegaMax: " << omegaMax_.transpose() << " [rad/s]" << std::endl
            << "omegaB0_B: " << omegaB0_0_.transpose() << " [rad/s]"
            << std::endl
            << "signs: " << s_.transpose() << std::endl
            << "elliptic modulus: " << k_ << std::endl
            << "angular momentum mag: " << h_ << std::endl
            << "rot kinetic energy: " << Ek_ << std::endl
            << std::endl
            << "R_WH: " << std::endl
            << R_WH_ << std::endl;
}

/* ************************************************************************** */
Eigen::Vector3d RigidBodyRotation::PredictOmega(const double t) const {
  Eigen::Vector3d omegaB_B{0.0, 0.0, 0.0};

  // Multi-axis rotation.
  if (symmetry_ == InertiaSymmetry::TA) {  // Tri-axis symmetry.
    // Get the elliptic integral terms.
    double sn, cn, dn;
    sn = boost::math::jacobi_elliptic(k_, omegaP_ * (t - T0_), &cn, &dn);

    // Get the angular velocity at time t.
    if (energy_ == EnergyState::LE || energy_ == EnergyState::ME) {
      // clang-format off
      omegaB_B = Eigen::Vector3d{s_(0) * omegaMax_(0) * dn,
                                 s_(1) * omegaMax_(1) * sn,
                                 s_(2) * omegaMax_(2) * cn};
    } else {  // HE
      omegaB_B = Eigen::Vector3d{s_(0) * omegaMax_(0) * cn,
                                 s_(1) * omegaMax_(1) * sn,
                                 s_(2) * omegaMax_(2) * dn};
      // clang-format on
    }

  } else if (symmetry_ == InertiaSymmetry::AS1) {  // Axis-symmetric J1,J2.
    std::cout << "RigidBodyRotation::PredictOmega" << std::endl;
    std::cout << "InertiaSymmetry::AS1 not implemented." << std::endl;
  } else if (symmetry_ == InertiaSymmetry::AS3) {  // Axis-symmetric J2,J3.
    std::cout << "RigidBodyRotation::PredictOmega" << std::endl;
    std::cout << "InertiaSymmetry::AS3 not implemented." << std::endl;
  }

  return omegaB_B;
}

/* ************************************************************************** */
Eigen::MatrixXd RigidBodyRotation::PredictOmega(
    const Eigen::VectorXd& times) const {
  Eigen::MatrixXd omegaB_B = Eigen::MatrixXd::Zero(3, times.size());
  for (size_t i = 0; i < times.size(); i++) {
    omegaB_B.block(0, i, 3, 1) = PredictOmega(times(i));
  }
  return omegaB_B;
}

/* ************************************************************************** */
Eigen::Matrix3d RigidBodyRotation::PredictOrientation(const double t) const {
  // Multi-axis rotation.
  Eigen::Matrix3d R_BHt = CalculateRotBH(t);
  Eigen::Matrix3d R_WBt = R_WH_ * R_BHt.transpose();  // R_WH * R_HB = R_WB

  return R_WBt;
}

/* ************************************************************************** */
std::vector<Eigen::Matrix3d> RigidBodyRotation::PredictOrientation(
    const Eigen::VectorXd& times) const {
  std::vector<Eigen::Matrix3d> Rs;
  size_t N = times.size();
  Rs.reserve(N);

  for (size_t i = 0; i < N; i++) Rs.push_back(PredictOrientation(times(i)));

  return Rs;
}

}  // namespace sph
