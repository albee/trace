/**
 * @file PrincipalAxesOpt.cpp
 * @brief Object to estimate principal axes of rigid body.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/Conics.h"

#include <Eigen/LU>

using namespace sph;

namespace sph {

/* ************************************************************************** */
Conic::Conic(const double A, const double B, const double C, const double D,
             const double E, const double F)
    : A_(A),
      B_(B),
      C_(C),
      D_(D),
      E_(E),
      F_(F),
      K_(ParametricConic(A_, B_, C_, D_, E_, F_)) {
  Kc_ = FindCanonical(K_, &T_AC_);
  geoC_ = GetCanonicalGeometry(Kc_);
}

/* ************************************************************************** */
Eigen::MatrixXd Conic::EvaluateCanonical(const std::vector<double>& angles) {
  Eigen::VectorXd vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      angles.data(), angles.size());
  return EvaluateCanonical(vec);
}

/* ************************************************************************** */
Eigen::MatrixXd Conic::EvaluateCanonical(const Eigen::VectorXd& angles) {
  // Create matrix to hold all evaluated points.
  size_t N = angles.size();
  Eigen::MatrixXd pts = Eigen::MatrixXd::Zero(2, N);

  if (geoC_.type == ConicType::ELLIPSE) {
    for (size_t i = 0; i < N; i++) {
      pts(0, i) = geoC_.aSemi * std::cos(angles(i));  // X-coordinate.
      pts(1, i) = geoC_.bSemi * std::sin(angles(i));  // Y-coordinate.
    }

  } else if (geoC_.type == ConicType::HYPERBOLA) {
    if (geoC_.dir == ConicDirection::XY) {
      for (size_t i = 0; i < N; i++) {
        pts(0, i) = geoC_.aSemi * std::cosh(angles(i));  // X-coordinate.
        pts(1, i) = geoC_.bSemi * std::sinh(angles(i));  // Y-coordinate.
      }

    } else {  // ConicDirection::YX
      for (size_t i = 0; i < N; i++) {
        pts(0, i) = geoC_.aSemi * std::sinh(angles(i));  // X-coordinate.
        pts(1, i) = geoC_.bSemi * std::cosh(angles(i));  // Y-coordinate.
      }
    }
  }

  return pts;
}

/* ************************************************************************** */
Eigen::MatrixXd Conic::Evaluate(const std::vector<double>& angles) {
  Eigen::VectorXd vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      angles.data(), angles.size());
  return Evaluate(vec);
}

/* ************************************************************************** */
Eigen::MatrixXd Conic::Evaluate(const Eigen::VectorXd& angles) {
  Eigen::MatrixXd pts_C = EvaluateCanonical(angles);

  // Rotate and translate points to actual parametric frame.
  Eigen::Matrix2d R_AC = T_AC_.block<2, 2>(0, 0);
  Eigen::Vector2d t_CwrtA_A = T_AC_.block<2, 1>(0, 2);
  Eigen::MatrixXd pts_A = R_AC * pts_C + t_CwrtA_A.replicate(1, angles.size());

  return pts_A;
}

/* ************************************************************************** */
ParametricConic FindCanonical(const ParametricConic& K, Eigen::Matrix3d* tfm) {
  // Unpack for convenience.
  double aA = K.A, bA = K.B, cA = K.C, dA = K.D, eA = K.E, fA = K.F;

  // Find the center of the conic in actual coordinates.
  Eigen::Matrix2d Amat;
  // clang-format off
  Amat <<   aA, bA/2,
          bA/2,   cA;
  // clang-format on
  Eigen::Matrix2d AmatLin = -2 * Amat;
  Eigen::Vector2d bVec{dA, eA};
  Eigen::Vector2d t_CwrtA_A = AmatLin.lu().solve(bVec);

  // Find rotation matrix between the canonical and actual frames.
  double theta = 0.5 * std::atan(bA / (aA - cA));
  Eigen::Matrix2d R_AC;
  // clang-format off
  R_AC << std::cos(theta), -std::sin(theta),
          std::sin(theta),  std::cos(theta);
  // clang-format on

  // Form 2D pose.
  Eigen::Matrix3d T_AC = Eigen::Matrix3d::Identity();
  T_AC.block<2, 2>(0, 0) = R_AC;       // Set rotation.
  T_AC.block<2, 1>(0, 2) = t_CwrtA_A;  // Set translation.
  if (tfm) {
    *tfm = T_AC;
  }

  // Get coefficients for the de-translated and de-rotated conics.
  double aC = aA * std::pow(std::cos(theta), 2) +
              bA * std::cos(theta) * std::sin(theta) +
              cA * std::pow(std::sin(theta), 2);
  double bC = 0;
  double cC = aA * std::pow(std::sin(theta), 2) -
              bA * std::sin(theta) * std::cos(theta) +
              cA * std::pow(std::cos(theta), 2);
  double dC = 0, eC = 0;
  double fC =
      fA - (aA * std::pow(t_CwrtA_A(0), 2) + bA * t_CwrtA_A(0) * t_CwrtA_A(1) +
            cA * std::pow(t_CwrtA_A(1), 2));
  if (fC != 0.0) {
    aC = -aC / fC;
    bC = -bC / fC;
    cC = -cC / fC;
    dC = -dC / fC;
    eC = -eC / fC;
    fC = -fC / fC;
  }

  // Bundle all together.
  return ParametricConic(aC, bC, cC, dC, eC, fC);
}

/* ************************************************************************** */
ConicCanonicalGeometry GetCanonicalGeometry(const ParametricConic& Kc) {
  if (!IsCanonical(Kc)) {
    std::cout << "GetCanonicalGeometry, not canonical..." << std::endl;
    std::exit(1);
  }
  ConicCanonicalGeometry geo;

  // Unpack values for convenience.
  double aC = Kc.A, cC = Kc.C, fC = Kc.F;

  // Get the semi-major axes, conic type, and direction.
  geo.aSemi = std::sqrt(std::abs(-fC / aC));
  geo.bSemi = std::sqrt(std::abs(-fC / cC));
  if (aC * cC > 0) {
    geo.type = ConicType::ELLIPSE;
    geo.dir = ConicDirection::XY;
    geo.ecc = std::sqrt(1 - std::min(-fC / aC, -fC / cC) /
                                std::max(-fC / aC, -fC / cC));
  } else if (aC * cC < 0) {
    geo.type = ConicType::HYPERBOLA;
    geo.ecc = std::sqrt(1 + std::abs(std::min(-fC / aC, -fC / cC)) /
                                std::abs(std::max(-fC / aC, -fC / cC)));
    if (aC > 0 && cC < 0) {
      geo.dir = ConicDirection::XY;
    } else if (aC < 0 && cC > 0) {
      geo.dir = ConicDirection::YX;
    }
  } else {
    std::cout << "FindCanonical, Parabola case..." << std::endl;
    std::exit(1);
  }

  return geo;
}

/* ************************************************************************** */
bool IsCanonical(const ParametricConic& Kc) {
  return (Kc.B == 0.0 && Kc.D == 0.0 && Kc.E == 0.0 && Kc.F == -1.0);
}

/* ************************************************************************** */
ConicFit<double> FitBestConicCanonical(const Eigen::MatrixXd& pts) {
  ConicFit<double> ellFit, hypFit;
  std::pair<ConicFit<double>, ConicFit<double>> fitPair =
      FitConicCanonical(pts);
  ellFit = fitPair.first;
  hypFit = fitPair.second;

  return ((ellFit.residual < hypFit.residual) ? ellFit : hypFit);
}

/* ************************************************************************** */
std::pair<ConicFit<double>, ConicFit<double>> FitConicCanonical(
    const Eigen::MatrixXd& pts) {
  // Square the entires.
  Eigen::MatrixXd pts2 = pts.cwiseProduct(pts);

  // Build the data matrix using the provided (x,y) points.
  const int N = pts.cols();
  Eigen::MatrixXd data = Eigen::MatrixXd::Ones(N, 3);
  Eigen::MatrixXd pts2T = pts2.transpose();
  data.block(0, 0, N, 2) = pts2.transpose();

  // Solve generalized eigenvalue problem.
  Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;
  ges.compute(data.transpose() * data, CeigConst<double>);
  return ParseEigenPair(ges.eigenvectors(), ges.alphas(), ges.betas());
}

/* ************************************************************************** */
std::pair<ConicFit<double>, ConicFit<double>> ParseEigenPair(
    const EigenvectorsType<double>& eigenvecs,
    const ComplexVectorType<double>& alphas, const VectorType<double>& betas) {
  // Get the position of infinite eigenvalue, ellipse, and hyperbola.
  size_t nan_pos, ell_pos, hyp_pos;
  double ell_lambda, hyp_lambda;
  for (size_t i = 0; i < 3; i++) {
    double eigval = alphas(i).real() / betas(i);
    if (std::isinf(eigval)) {
      nan_pos = i;
    } else if (eigval > 0) {  // Positive eigenvalue corresponds to ellipse.
      ell_lambda = eigval;
      ell_pos = i;
    } else {  // Negative eigenvalue corresponds to hyperbola.
      hyp_lambda = eigval;
      hyp_pos = i;
    }
  }

  // Build the conics.
  ConicFit<double> ellFit, hypFit;

  ellFit.residual = std::abs(ell_lambda);
  ellFit.eigv = Eigen::Vector3d{eigenvecs(0, ell_pos).real(),
                                eigenvecs(1, ell_pos).real(),
                                eigenvecs(2, ell_pos).real()};
  Eigen::Vector3d KcEll = -ellFit.eigv / ellFit.eigv(2);          // [A C F].
  ellFit.c = Conic(KcEll(0), 0.0, KcEll(1), 0.0, 0.0, KcEll(2));  // K: [all]

  hypFit.residual = std::abs(hyp_lambda);
  hypFit.eigv = Eigen::Vector3d{eigenvecs(0, hyp_pos).real(),
                                eigenvecs(1, hyp_pos).real(),
                                eigenvecs(2, hyp_pos).real()};
  Eigen::Vector3d KcHyp = -hypFit.eigv / hypFit.eigv(2);          // [A C F].
  hypFit.c = Conic(KcHyp(0), 0.0, KcHyp(1), 0.0, 0.0, KcHyp(2));  // K: [all]

  return std::make_pair(ellFit, hypFit);
}

}  // namespace sph
