/**
 * @file Conics.tpp
 * @brief Functions related to conic sections.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2021 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_CONICS_TPP_
#define SPH_VERTIGO_CONICS_TPP_

namespace sph {

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
      K_(ParametricConic<T>(A_, B_, C_, D_, E_, F_)) {
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

  if (geoC_.type == ConicType::ELLIPSE) {
    for (size_t i = 0; i < N; i++) {
      pts(0, i) = geoC_.aSemi * cos(angles(i));  // X-coordinate.
      pts(1, i) = geoC_.bSemi * sin(angles(i));  // Y-coordinate.
    }

  } else if (geoC_.type == ConicType::HYPERBOLA) {
    if (geoC_.dir == ConicDirection::XY) {
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

  // Bundle all together.
  return ParametricConic<T>(aC, bC, cC, dC, eC, fC);
}

/* ************************************************************************** */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> FitConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts) {
  // Square the entires.
  Eigen::Matrix<T, 2, Eigen::Dynamic> pts2 = pts.cwiseProduct(pts);

  // Build the data matrix using the provided (x,y) points.
  const int N = pts.cols();
  Eigen::Matrix<T, Eigen::Dynamic, 3> data =
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(N, 3);
  Eigen::Matrix<T, Eigen::Dynamic, 2> pts2T = pts2.transpose();
  data.block(0, 0, N, 2) = pts2.transpose();

  const Eigen::Matrix<T, 3, 3> D = data.transpose() * data;

  // Get the data's coefficients for convenience.
  const T xx = D(0, 0);
  const T xy = D(0, 1);
  const T x1 = D(0, 2);
  const T yy = D(1, 1);
  const T y1 = D(1, 2);

  // Terms for the 2x2 eigenvalue problem.
  const T alpha11 = T(2.0) * (xy - ((T(1) / T(N)) * x1 * y1));
  const T alpha12 = T(2.0) * (yy - ((T(1) / T(N)) * y1 * y1));
  const T alpha21 = T(2.0) * (xx - ((T(1) / T(N)) * x1 * x1));

  // Compute the analytical solution to the generalized eigenvalue problem.
  const T ell_lambda = alpha11 + sqrt(alpha12 * alpha21);
  const T hyp_lambda = alpha11 - sqrt(alpha12 * alpha21);

  const T ell_z2 = T(1.0);
  const T ell_z1 = -((alpha11 - ell_lambda) / (alpha21)) * ell_z2;
  const T ell_z3 = (-D(2, 0) * ell_z1 - D(2, 1) * ell_z2) / D(2, 2);

  const T hyp_z2 = T(1.0);
  const T hyp_z1 = -((alpha11 - hyp_lambda) / (alpha21)) * hyp_z2;
  const T hyp_z3 = (-D(2, 0) * hyp_z1 - D(2, 1) * hyp_z2) / D(2, 2);


  // And bundle everything together.
  //const ConicFit<T> ell = BuildConicFit(D, lambda_ell, alpha11, alpha21);
  //const ConicFit<T> hyp = BuildConicFit(D, lambda_hyp, alpha11, alpha21);


  ConicFit<T> ellFit, hypFit;

  ellFit.eigvec = Eigen::Matrix<T, 3, 1>(ell_z1 / -ell_z3, ell_z2 / -ell_z3, T(-1.0));
  ellFit.eigv = ellFit.eigvec;
  ellFit.residual = fabs(ell_lambda);
  //ellFit.eigv = Eigen::Matrix<T, 3, 1>{eigenvecs(0, ell_pos).real(),
  //                                     eigenvecs(1, ell_pos).real(),
  //                                     eigenvecs(2, ell_pos).real()};
  Eigen::Matrix<T, 3, 1> KcEll = -ellFit.eigv / ellFit.eigv(2);  // [A C F].
  ellFit.c = Conic<T>(KcEll(0), T(0.0), KcEll(1), T(0.0), T(0.0),
                      KcEll(2));  // K: [all]


  hypFit.eigvec = Eigen::Matrix<T, 3, 1>(hyp_z1 / -hyp_z3, hyp_z2 / -hyp_z3, T(-1.0));
  hypFit.eigv = hypFit.eigvec;
  hypFit.residual = fabs(hyp_lambda);
  //hypFit.eigv = Eigen::Matrix<T, 3, 1>{eigenvecs(0, hyp_pos).real(),
  //                                     eigenvecs(1, hyp_pos).real(),
  //                                     eigenvecs(2, hyp_pos).real()};
  Eigen::Matrix<T, 3, 1> KcHyp = -hypFit.eigv / hypFit.eigv(2);  // [A C F].
  hypFit.c = Conic<T>(KcHyp(0), T(0.0), KcHyp(1), T(0.0), T(0.0),
                      KcHyp(2));  // K: [all]

  return std::make_pair(ellFit, hypFit);



  //return {ell, hyp};

  // Solve generalized eigenvalue problem.
  //Eigen::GeneralizedEigenSolver<
  //    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>
  //    ges;
  //ges.compute(data.transpose() * data, CeigConst<T>);
  //return ParseEigenPair<T>(ges.eigenvectors(), ges.alphas(), ges.betas());
}

template <typename T>
ConicFit<T> BuildConicFit(const Eigen::Matrix<T, 3, 3>& D, const T& lambda,
                          const T& alpha11, const T& alpha21) {
  ConicFit<T> fit;
  fit.residual_square = fabs(lambda);
  fit.residual = fit.residual_square;

  // Analytical solution to GEVP eigenvector.
  const T z2 = T(1.0);
  const T z1 = -((alpha11 - lambda) / (alpha21)) * z2;
  const T z3 = (-D(2, 0) * z1 - D(2, 1) * z2) / D(2, 2);

  fit.eigvec = Eigen::Matrix<T, 3, 1>(z1 / -z3, z2 / -z3, T(-1.0));
  fit.eigv = fit.eigvec;
  /*
  fit.K = {.A = fit.eigvec(0),
           .B = T(0.0),
           .C = fit.eigvec(1),
           .D = T(0.0),
           .E = T(0.0),
           .F = fit.eigvec(2)};
  */
  fit.K.A = fit.eigvec(0),
  fit.K.B = T(0.0),
  fit.K.C = fit.eigvec(1),
  fit.K.D = T(0.0),
  fit.K.E = T(0.0),
  fit.K.F = fit.eigvec(2);
  return fit;
}

/* ************************************************************************** */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> ParseEigenPair(
    const EigenvectorsType<T>& eigenvecs, const ComplexVectorType<T>& alphas,
    const VectorType<T>& betas) {
  // Get the position of infinite eigenvalue, ellipse, and hyperbola.
  size_t nan_pos, ell_pos, hyp_pos;
  T ell_lambda, hyp_lambda;
  for (size_t i = 0; i < 3; i++) {
    T eigval = alphas(i).real() / betas(i);
    if (std::isinf(eigval)) {
      nan_pos = i;
    } else if (eigval > T(0)) {  // Positive eigenvalue corresponds to ellipse.
      ell_lambda = eigval;
      ell_pos = i;
    } else {  // Negative eigenvalue corresponds to hyperbola.
      hyp_lambda = eigval;
      hyp_pos = i;
    }
  }

  // Build the conics.
  ConicFit<T> ellFit, hypFit;

  ellFit.residual = fabs(ell_lambda);
  ellFit.eigv = Eigen::Matrix<T, 3, 1>{eigenvecs(0, ell_pos).real(),
                                       eigenvecs(1, ell_pos).real(),
                                       eigenvecs(2, ell_pos).real()};
  Eigen::Matrix<T, 3, 1> KcEll = -ellFit.eigv / ellFit.eigv(2);  // [A C F].
  ellFit.c = Conic<T>(KcEll(0), T(0.0), KcEll(1), T(0.0), T(0.0),
                      KcEll(2));  // K: [all]

  hypFit.residual = fabs(hyp_lambda);
  hypFit.eigv = Eigen::Matrix<T, 3, 1>{eigenvecs(0, hyp_pos).real(),
                                       eigenvecs(1, hyp_pos).real(),
                                       eigenvecs(2, hyp_pos).real()};
  Eigen::Matrix<T, 3, 1> KcHyp = -hypFit.eigv / hypFit.eigv(2);  // [A C F].
  hypFit.c = Conic<T>(KcHyp(0), T(0.0), KcHyp(1), T(0.0), T(0.0),
                      KcHyp(2));  // K: [all]

  return std::make_pair(ellFit, hypFit);
}

/* ************************************************************************** */
template <typename T>
bool IsCanonical(const ParametricConic<T>& Kc) {
  return (Kc.B == T(0.0) && Kc.D == T(0.0) && Kc.E == T(0.0) &&
          Kc.F == T(-1.0));
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
  T aC = (float)Kc.A, cC = (float)Kc.C, fC = (float)Kc.F;

  // Get the semi-major axes, conic type, and direction.
  geo.aSemi = sqrt(fabs(-fC / aC));
  geo.bSemi = sqrt(fabs(-fC / cC));
  if (aC * cC > T(0)) {
    geo.type = ConicType::ELLIPSE;
    geo.dir = ConicDirection::XY;
    geo.ecc = sqrt(T(1) -
                   std::min(-fC / aC, -fC / cC) / std::max(-fC / aC, -fC / cC));
  } else if (aC * cC < T(0)) {
    geo.type = ConicType::HYPERBOLA;
    // std::cout << aC << " " << cC << " " << fC << std::endl;
    // std::cout << T(1) << std::endl;
    // std::cout << std::min(-fC / aC, -fC / cC) << std::endl;
    // std::cout << std::max(-fC / aC, -fC / cC) << std::endl;
    // std::cout << fabs(std::min(-fC / aC, -fC / cC)) << std::endl;
    // std::cout << fabs(std::max(-fC / aC, -fC / cC)) << std::endl;
    // std::cout << fabs(std::min(-fC / aC, -fC / cC)) / fabs(std::max(-fC / aC, -fC / cC));
    // std::cout << sqrt(T(1) + fabs(std::min(-fC / aC, -fC / cC)) /
    //                           fabs(std::max(-fC / aC, -fC / cC))) << std::endl;
    geo.ecc = sqrt(T(1) + fabs(std::min(-fC / aC, -fC / cC)) /
                              fabs(std::max(-fC / aC, -fC / cC)));
    if (aC > T(0) && cC < T(0)) {
      geo.dir = ConicDirection::XY;
    } else if (aC < T(0) && cC > T(0)) {
      geo.dir = ConicDirection::YX;
    }
    std::cout << "fC: " << fC << std::endl;
    std::cout << "aC: " << aC << std::endl;
    std::cout << "cC: " << cC << std::endl;
    std::cout << "min: " << std::min(-fC / aC, -fC / cC) << std::endl;
    std::cout << "max: " << std::max(-fC / aC, -fC / cC) << std::endl;
    std::cout << "T(1): " << T(1) << std::endl;
  } else {
    std::cout << "FindCanonical, Parabola case..." << std::endl;
    std::exit(1);
  }

  return geo;
}

/* ************************************************************************** */
template <typename T>
ConicFit<T> FitBestConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts) {
  ConicFit<T> ellFit, hypFit;
  std::pair<ConicFit<T>, ConicFit<T>> fitPair = FitConicCanonical<T>(pts);
  ellFit = fitPair.first;
  hypFit = fitPair.second;

  return ((ellFit.residual < hypFit.residual) ? ellFit : hypFit);
}

/* ************************************************************************** */
template <typename T>
std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>> GetBestConicFits(
    const Matrix3<T>& R_EG, const Matrix3X<T>& G_omegaB) {
  const size_t N = G_omegaB.cols();
  // Rotate data to proposed elliptical frame E.
  Matrix3X<T> omegaB_E = R_EG * G_omegaB;

  // Splice the data by planes.
  Matrix2X<T> xyData(2, N), xzData(2, N), yzData(2, N);
  xyData.block(0, 0, 1, N) = omegaB_E.block(0, 0, 1, N);  // x to x
  xyData.block(1, 0, 1, N) = omegaB_E.block(1, 0, 1, N);  // y to y
  xzData.block(0, 0, 1, N) = omegaB_E.block(0, 0, 1, N);  // x to x
  xzData.block(1, 0, 1, N) = omegaB_E.block(2, 0, 1, N);  // z to y
  yzData.block(0, 0, 1, N) = omegaB_E.block(1, 0, 1, N);  // y to x
  yzData.block(1, 0, 1, N) = omegaB_E.block(2, 0, 1, N);  // z to y

  // Get best fits in each plane.
  ConicFit<T> xyFit = FitBestConicCanonical<T>(xyData);
  ConicFit<T> xzFit = FitBestConicCanonical<T>(xzData);
  ConicFit<T> yzFit = FitBestConicCanonical<T>(yzData);

  return std::make_tuple(xyFit, xzFit, yzFit);
}

}  // namespace sph

#endif  // SPH_VERTIGO_CONICS_TPP_
