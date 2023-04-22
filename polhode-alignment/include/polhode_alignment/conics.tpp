/**
 * @file Conics.tpp
 * @brief Functions related to conic sections.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#ifndef POLHODE_ALIGNMENT_CONICS_TPP_
#define POLHODE_ALIGNMENT_CONICS_TPP_

#include <iostream>

using std::abs;

namespace pao {

/* ************************************************************************** */
template <typename T>
Conic<T>::Conic(const T A, const T B, const T C, const T D, const T E,
                const T F)
    : A_(A),
      B_(B),
      C_(C),
      D_(D),
      E_(E),
      F_(F),
      K_(ParametricConic<T>{A_, B_, C_, D_, E_, F_}) {
  Kc_ = FindCanonical<T>(K_, &T_AC_);
  geoC_ = GetCanonicalGeometry<T>(Kc_);
}

/* ************************************************************************** */
template <typename T>
Eigen::Matrix<T, 2, Eigen::Dynamic> Conic<T>::EvaluateCanonical(
    const std::vector<T>& angles) {
  Eigen::Matrix<T, Eigen::Dynamic, 1> vec =
      Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>, Eigen::Unaligned>(
          angles.data(), angles.size());
  return EvaluateCanonical(vec);
}

/* ************************************************************************** */
template <typename T>
Eigen::Matrix<T, 2, Eigen::Dynamic> Conic<T>::EvaluateCanonical(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& angles) {
  // Create matrix to hold all evaluated points.
  size_t N = angles.size();
  Eigen::Matrix<T, 2, Eigen::Dynamic> pts =
      Eigen::Matrix<T, 2, Eigen::Dynamic>::Zero(2, N);

  if (geoC_.type == ConicType::kEllipse) {
    for (size_t i = 0; i < N; i++) {
      pts(0, i) = geoC_.aSemi * cos(angles(i));  // X-coordinate.
      pts(1, i) = geoC_.bSemi * sin(angles(i));  // Y-coordinate.
    }

  } else if (geoC_.type == ConicType::kHyperbola) {
    if (geoC_.dir == ConicDirection::kXY) {
      for (size_t i = 0; i < N; i++) {
        pts(0, i) = geoC_.aSemi * cosh(angles(i));  // X-coordinate.
        pts(1, i) = geoC_.bSemi * sinh(angles(i));  // Y-coordinate.
      }

    } else {  // ConicDirection::YX
      for (size_t i = 0; i < N; i++) {
        pts(0, i) = geoC_.aSemi * sinh(angles(i));  // X-coordinate.
        pts(1, i) = geoC_.bSemi * cosh(angles(i));  // Y-coordinate.
      }
    }
  }

  return pts;
}

/* ************************************************************************** */
template <typename T>
Eigen::Matrix<T, 2, Eigen::Dynamic> Conic<T>::Evaluate(
    const std::vector<T>& angles) {
  Eigen::Matrix<T, Eigen::Dynamic, 1> vec =
      Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>, Eigen::Unaligned>(
          angles.data(), angles.size());
  return Evaluate(vec);
}

/* ************************************************************************** */
template <typename T>
Eigen::Matrix<T, 2, Eigen::Dynamic> Conic<T>::Evaluate(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& angles) {
  Eigen::Matrix<T, 2, Eigen::Dynamic> pts_C = EvaluateCanonical(angles);

  // Rotate and translate points to actual parametric frame.
  Eigen::Matrix<T, 2, 2> R_AC = T_AC_.block(0, 0, 2, 2);
  Eigen::Matrix<T, 2, 1> t_CwrtA_A = T_AC_.block(0, 2, 2, 1);
  Eigen::Matrix<T, 2, Eigen::Dynamic> pts_A =
      R_AC * pts_C + t_CwrtA_A.replicate(1, angles.size());

  return pts_A;
}

/* ************************************************************************** */
template <typename T>
ParametricConic<T> FindCanonical(const ParametricConic<T>& K,
                                 Eigen::Matrix<T, 3, 3>* tfm) {
  // Unpack for convenience.
  T aA = K.A, bA = K.B, cA = K.C, dA = K.D, eA = K.E, fA = K.F;

  // Find the center of the conic in actual coordinates.
  Eigen::Matrix<T, 2, 2> Amat;
  // clang-format off
  Amat <<      aA, bA/T(2),
          bA/T(2),      cA;
  // clang-format on
  Eigen::Matrix<T, 2, 2> AmatLin = T(-2) * Amat;
  Eigen::Matrix<T, 2, 1> bVec{dA, eA};
  Eigen::Matrix<T, 2, 1> t_CwrtA_A = AmatLin.lu().solve(bVec);

  // Find rotation matrix between the canonical and actual frames.
  T theta = T(0.5) * atan(bA / (aA - cA));
  Eigen::Matrix<T, 2, 2> R_AC;
  // clang-format off
  R_AC << cos(theta), -sin(theta),
          sin(theta),  cos(theta);
  // clang-format on

  // Form 2D pose.
  Eigen::Matrix<T, 3, 3> T_AC = Eigen::Matrix<T, 3, 3>::Identity();
  T_AC.block(0, 0, 2, 2) = R_AC;       // Set rotation.
  T_AC.block(0, 2, 2, 1) = t_CwrtA_A;  // Set translation.
  if (tfm) {
    *tfm = T_AC;
  }

  // Get coefficients for the de-translated and de-rotated conics.
  T aC = aA * pow(cos(theta), 2) + bA * cos(theta) * sin(theta) +
         cA * pow(sin(theta), 2);
  T bC = T(0);
  T cC = aA * pow(sin(theta), 2) - bA * sin(theta) * cos(theta) +
         cA * pow(cos(theta), 2);
  T dC = T(0), eC = T(0);
  T fC = fA - (aA * pow(t_CwrtA_A(0), 2) + bA * t_CwrtA_A(0) * t_CwrtA_A(1) +
               cA * pow(t_CwrtA_A(1), 2));
  if (fC != T(0.0)) {
    aC = -aC / fC;
    bC = -bC / fC;
    cC = -cC / fC;
    dC = -dC / fC;
    eC = -eC / fC;
    fC = -fC / fC;
  }

  return ParametricConic<T>{aC, bC, cC, dC, eC, fC};
}

/* ************************************************************************** */
template <typename T>
ConicCanonicalGeometry<T> GetCanonicalGeometry(const ParametricConic<T>& Kc) {
  if (!IsCanonical<T>(Kc)) {
    std::cout << "GetCanonicalGeometry, not canonical..." << std::endl;
    std::exit(1);
  }
  ConicCanonicalGeometry<T> geo;

  // Unpack values for convenience.
  T aC = Kc.A, cC = Kc.C, fC = Kc.F;

  // Get the semi-major axes, conic type, and direction.
  geo.aSemi = sqrt(abs(-fC / aC));
  geo.bSemi = sqrt(abs(-fC / cC));
  if (aC * cC > T(0)) {
    geo.type = ConicType::kEllipse;
    geo.dir = ConicDirection::kXY;
    geo.ecc = sqrt(T(1) -
                   std::min(-fC / aC, -fC / cC) / std::max(-fC / aC, -fC / cC));
  } else if (aC * cC < T(0)) {
    geo.type = ConicType::kHyperbola;
    geo.ecc = sqrt(T(1) + abs(std::min(-fC / aC, -fC / cC)) /
                              abs(std::max(-fC / aC, -fC / cC)));
    if (aC > T(0) && cC < T(0)) {
      geo.dir = ConicDirection::kXY;
    } else if (aC < T(0) && cC > T(0)) {
      geo.dir = ConicDirection::kYX;
    }
  } else {
    std::cout << "FindCanonical, Parabola case..." << std::endl;
    std::exit(1);
  }

  return geo;
}

/* ************************************************************************** */
template <typename T>
bool IsCanonical(const ParametricConic<T>& Kc) {
  return (Kc.B == T(0.0) && Kc.D == T(0.0) && Kc.E == T(0.0) &&
          Kc.F == T(-1.0));
}

}  // namespace pao

#endif  // POLHODE_ALIGNMENT_CONICS_TPP_
