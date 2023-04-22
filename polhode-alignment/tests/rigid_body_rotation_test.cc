/**
 * @file rigid_body_rotation_test.cc
 * @brief Unit tests for all rigid body dynamics.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#include "polhode_alignment/rigid_body_rotation.h"

#include <Eigen/Dense>
#include <boost/math/special_functions/ellint_1.hpp>
#include <boost/math/special_functions/ellint_3.hpp>
#include <boost/math/special_functions/jacobi_elliptic.hpp>
#include <memory>

#include "gtest/gtest.h"

namespace {

/* ************************************************************************** */
TEST(RigidBodyRotationTest, Construction) {
  pao::RigidBodyParams params;
  pao::RigidBodyRotation rbr{params};

  std::cout << "TEST TEST TEST" << std::endl;
  std::cout << boost::math::ellint_1(1.0, 1.57) << std::endl;

  double sn, cn, dn;
  sn = boost::math::jacobi_elliptic(0.6781 /*k*/, 0.1196 * 10 /*u*/, &cn, &dn);
  std::cout << "sn: " << sn << std::endl
            << "cn: " << cn << std::endl
            << "dn: " << dn << std::endl;

  // args: k, n, phi
  std::cout << boost::math::ellint_3(0.6781, 1, 2) << std::endl;
  std::cout << boost::math::ellint_3(0.6781, -1, 2) << std::endl;

  Eigen::Matrix3d R1;
  R1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  std::cout << R1 << std::endl;
}

/* ************************************************************************** */
TEST(RigidBodyRotationTest, ConstructionSPHERES) {
  pao::RigidBodyParams params;
  params.J = Eigen::Vector3d{1.239, 1.1905, 1.0}.asDiagonal();  // SPHERES
  params.R_WB0 = Eigen::Matrix3d::Identity();   // Initial orientation.
  params.init = pao::DynamicsInitType::OMEGA0;  // Initialize with omega0;
  // HE case.
  params.omega0 = Eigen::Vector3d{0, 0.939392242898362, 0.500486277097766};
  pao::RigidBodyRotation rbr{params};

  // Compare against MATLAB values.
  EXPECT_EQ(rbr.J1(), 1.239);  // Inertia ratios.
  EXPECT_EQ(rbr.J2(), 1.1905);
  EXPECT_EQ(rbr.J3(), 1.0);
  // EXPECT_TRUE(rbt.omegaB0_0_.isApprox(Eigen::Vector3d{0, 0.9394, 0.5004}));
  double epsilon = 1e-4;
  EXPECT_NEAR(rbr.Ek(), 0.6505, epsilon);   // Rotational kinetic energy.
  EXPECT_NEAR(rbr.h(), 1.2252, epsilon);    // Angular momentum magnitude.
  EXPECT_NEAR(rbr.T0(), -15.225, epsilon);  // Initial experiment time.

  Eigen::Matrix3d R_WHmatlab;
  // clang-format off
  R_WHmatlab << -0.3030,   -0.9530,   -0.0000,
                 0.3893,   -0.1238,    0.9128,
                -0.8698,    0.2766,    0.4085;
  // clang-format on
  EXPECT_TRUE(rbr.R_WH().isApprox(R_WHmatlab, 1e-3));

  EXPECT_EQ(rbr.s()(0), -1);  // Signs by convention.
  EXPECT_EQ(rbr.s()(1), 1);
  EXPECT_EQ(rbr.s()(2), 1);

  EXPECT_NEAR(rbr.T(), 15.2250, epsilon);  // Initial time.
  EXPECT_TRUE(rbr.omegaMax().isApprox(Eigen::Vector3d{0.8221, 0.9394, 0.6809},
                                      epsilon));
  EXPECT_NEAR(rbr.k(), 0.6781, epsilon);       // Elliptic modulus.
  EXPECT_NEAR(rbr.omegaP(), 0.1196, epsilon);  // Nutation rate.
  EXPECT_EQ(rbr.Energy(), pao::EnergyState::HE);
  EXPECT_EQ(rbr.Symmetry(), pao::InertiaSymmetry::TA);
}

/* ************************************************************************** */
TEST(RigidBodyRotationTest, SelfConsistencyCheck) {
  pao::RigidBodyParams params;
  params.J = Eigen::Vector3d{1.239, 1.1905, 1.0}.asDiagonal();  // SPHERES
  params.R_WB0 = Eigen::Matrix3d::Identity();   // Initial orientation.
  params.init = pao::DynamicsInitType::OMEGA0;  // Initialize with omega0;
  // HE case.
  params.omega0 = Eigen::Vector3d{0, 0.939392242898362, 0.500486277097766};
  pao::RigidBodyRotation rbr{params};

  // Create here intializing using energy and angular momentum.
  pao::RigidBodyParams params2;
  params2.J = Eigen::Vector3d{rbr.J1(), rbr.J2(), rbr.J3()}.asDiagonal();
  params2.init = pao::DynamicsInitType::ENERGY;
  params2.Ek = rbr.Ek();
  params2.h = rbr.h();
  params2.T0 = rbr.T0();
  pao::RigidBodyRotation rbr2{params2};

  // Ensure predictions are the same.
  double totalErrorOmega = 0;
  double totalErrorRot = 0;
  for (double t = 0.0; t < rbr.T() * 4; t += 0.5) {
    // Check omegas.
    Eigen::Vector3d omegaBt_B1 = rbr.PredictOmega(t);
    Eigen::Vector3d omegaBt_B2 = rbr2.PredictOmega(t);
    totalErrorOmega += (omegaBt_B2 - omegaBt_B1).norm();

    Eigen::Matrix3d R_WB1 = rbr.PredictOrientation(t);
    Eigen::Matrix3d R_WB2 = rbr2.PredictOrientation(t);
    totalErrorRot += (R_WB2 - R_WB1).norm();
  }
  EXPECT_NEAR(totalErrorOmega, 0.0, 1e-6);
  EXPECT_NEAR(totalErrorRot, 0.0, 1e-6);
}

/* ************************************************************************** */
TEST(RigidBodyRotationTest, MultiTimePrediction) {
  pao::RigidBodyParams params;
  params.J = Eigen::Vector3d{1.239, 1.1905, 1.0}.asDiagonal();  // SPHERES
  params.R_WB0 = Eigen::Matrix3d::Identity();   // Initial orientation.
  params.init = pao::DynamicsInitType::OMEGA0;  // Initialize with omega0;
  params.omega0 = Eigen::Vector3d{0, 0.939392242898362, 0.500486277097766};
  pao::RigidBodyRotation rbr{params};
  rbr.Print();

  int N = 200;
  Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(N, 0.0, 2.5 * rbr.T());

  Eigen::MatrixXd omegasB_B = rbr.PredictOmega(times);
  std::vector<Eigen::Matrix3d> R_WBt = rbr.PredictOrientation(times);
  for (size_t i = 0; i < times.size(); i++) {
    EXPECT_TRUE(
        omegasB_B.block(0, i, 3, 1).isApprox(rbr.PredictOmega(times(i)), 1e-4));
    // std::cout << omegasB_B.block(0, i, 3, 1).transpose() << std::endl;
    // std::cout << rbr.PredictOmega(times(i)).transpose() << std::endl;
    EXPECT_TRUE(R_WBt[i].isApprox(rbr.PredictOrientation(times(i)), 1e-4));
    // std::cout << "R_WBt: \n" << R_WBt[i] << std::endl;
    // std::cout << "Pred : \n" << rbr.PredictOrientation(times(i)) <<
    // std::endl;
  }
}

}  // namespace
