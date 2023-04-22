/**
 * @file conics_test.cc
 * @brief Unit tests for conic related functions.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#include "polhode_alignment/conics.h"

#include <Eigen/Dense>
#include <memory>

#include "gtest/gtest.h"
#include "polhode_alignment/geometry_utils.h"

using T = double;

namespace {

TEST(ConicsTest, Construction) {
  // Create a dummy canonical conic.
  pao::ParametricConic<T> Kc = {
      .A = 1, .B = 0, .C = 2, .D = 0, .E = 0, .F = -1};
  EXPECT_TRUE(pao::IsCanonical<T>(Kc));

  pao::Conic<T> c(1, 0, 2, 0, 0, -1);
  EXPECT_EQ(Kc, c.Kc());
  EXPECT_EQ(c.Geo(), pao::GetCanonicalGeometry<T>(Kc));

  std::cout << c << std::endl;
}

TEST(ConicsTest, MatlabComparison) {
  // Create a dummy conic.
  pao::ParametricConic<T> K{-1, 12, 2, -10, -10, -1};
  EXPECT_FALSE(pao::IsCanonical<T>(K));

  pao::Conic<T> c(-1, 12, 2, -10, -10, -1);
  EXPECT_EQ(K, c.K());
  std::cout << c << std::endl;

  double epsilon = 1e-4;

  // Compare canonical conic.
  pao::ParametricConic<T> KcMatlab{-0.6902, 0, 0.8116, 0, 0, -1.0};
  EXPECT_NEAR(c.Kc().A, KcMatlab.A, epsilon);
  EXPECT_NEAR(c.Kc().B, KcMatlab.B, epsilon);
  EXPECT_NEAR(c.Kc().C, KcMatlab.C, epsilon);
  EXPECT_NEAR(c.Kc().D, KcMatlab.D, epsilon);
  EXPECT_NEAR(c.Kc().E, KcMatlab.E, epsilon);
  EXPECT_NEAR(c.Kc().F, KcMatlab.F, epsilon);

  // Compare against matlab values.
  EXPECT_NEAR(c.a(), 1.2037, epsilon);
  EXPECT_NEAR(c.b(), 1.1100, epsilon);
  EXPECT_NEAR(c.ecc(), 1.4751, epsilon);
  EXPECT_EQ(c.Type(), pao::ConicType::kHyperbola);
  EXPECT_EQ(c.Direction(), pao::ConicDirection::kYX);

  // Compar transform between actual and canonical frames.
  Eigen::Matrix<T, 3, 3> T_ACmatlab;
  // clang-format off
  T_ACmatlab <<  0.7882,    0.6154,    0.5263,
                -0.6154,    0.7882,    0.9211,
                      0,         0,    1.0000;
  // clang-format on
  EXPECT_TRUE(c.T_AC().isApprox(T_ACmatlab, 1e-3));
}

}  // namespace
