/**
 * @file conic_fitting_test.cc
 * @brief Unit tests for conic fitting related functions.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#include "polhode_alignment/conic_fitting.h"

#include <Eigen/Dense>
#include <memory>
#include <random>

#include "gtest/gtest.h"
#include "polhode_alignment/geometry_utils.h"

using T = double;

namespace {

TEST(ConicsTest, ConicDataFit) {
  pao::ParametricConic<T> K{-1, 12, 2, -10, -10, -1};

  pao::Conic<T> c(-1, 12, 2, -10, -10, -1);
  std::cout << c << std::endl;

  // Generate canonical data.
  int N = 200;
  Eigen::Matrix<T, Eigen::Dynamic, 1> anglesPi =
      Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(N, -M_PI, M_PI);
  EXPECT_EQ(anglesPi.rows(), N);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ptsC =
      c.EvaluateCanonical(anglesPi);
  EXPECT_EQ(ptsC.rows(), 2);
  EXPECT_EQ(ptsC.cols(), N);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data =
      ptsC + pao::SampleNormalMatrix<T>(2, N, 0, 0.005);
  EXPECT_EQ(data.rows(), 2);
  EXPECT_EQ(data.cols(), N);

  // Try to recover the canonical conic for comparison purposes.
  //const auto [ellFit, hypFit] = pao::FitConicCanonical<T>(data);
  pao::ConicFit<T> ellFit;
  pao::ConicFit<T> hypFit;
  std::tie(ellFit, hypFit) = pao::FitConicCanonical<T>(data);

  std::cout << ellFit << std::endl;
  std::cout << hypFit << std::endl;

  double epsilon = 1e-2;
  EXPECT_NEAR(hypFit.K.A, c.Kc().A, epsilon);
  EXPECT_NEAR(hypFit.K.B, c.Kc().B, epsilon);
  EXPECT_NEAR(hypFit.K.C, c.Kc().C, epsilon);
  EXPECT_NEAR(hypFit.K.D, c.Kc().D, epsilon);
  EXPECT_NEAR(hypFit.K.E, c.Kc().E, epsilon);
  EXPECT_NEAR(hypFit.K.F, c.Kc().F, epsilon);
}

TEST(ConicsTest, BestDataFit) {
  pao::ParametricConic<T> K{-1, 12, 2, -10, -10, -1};
  pao::Conic<T> c(-1, 12, 2, -10, -10, -1);

  // Generate canonical data.
  int N = 200;
  Eigen::Matrix<T, Eigen::Dynamic, 1> anglesPi =
      Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(N, -M_PI, M_PI);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ptsC =
      c.EvaluateCanonical(anglesPi);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data =
      ptsC + pao::SampleNormalMatrix<T>(2, N, 0, 0.005);
  // Try to recover the canonical conic for comparison purposes.
  //const auto [ellFit, hypFit] = pao::FitConicCanonical<T>(data);
  pao::ConicFit<T> ellFit;
  pao::ConicFit<T> hypFit;
  std::tie(ellFit, hypFit) = pao::FitConicCanonical<T>(data);

  // Test the fitting of the best canonical conic.
  pao::ConicFit<T> bestFit = pao::FitBestConicCanonical<T>(data);
  EXPECT_EQ(bestFit.residual_square, hypFit.residual_square);
  EXPECT_TRUE(bestFit.eigvec.isApprox(hypFit.eigvec, 1e-4));
  EXPECT_EQ(bestFit.K, hypFit.K);
}

}  // namespace
