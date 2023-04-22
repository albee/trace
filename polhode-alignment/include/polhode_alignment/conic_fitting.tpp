/**
 * @file conic_fitting.tpp
 * @brief Functions related to conic fitting to 2D data points.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#ifndef POLHODE_ALIGNMENT_CONIC_FITTING_TPP_
#define POLHODE_ALIGNMENT_CONIC_FITTING_TPP_

using std::abs;

namespace pao {

namespace {

// Computes analytical solution for the GEVP eigenvectors. Scales them such that
// the central conic's F parameter (e.g., z3) is -1.
template <typename T>
ConicFit<T> BuildConicFit(const Eigen::Matrix<T, 3, 3>& D, const T& lambda,
                          const T& alpha11, const T& alpha21) {
  ConicFit<T> fit;
  fit.residual_square = abs(lambda);

  // Analytical solution to GEVP eigenvector.
  const T z2 = T(1.0);
  const T z1 = -((alpha11 - lambda) / (alpha21)) * z2;
  const T z3 = (-D(2, 0) * z1 - D(2, 1) * z2) / D(2, 2);

  fit.eigvec = Eigen::Matrix<T, 3, 1>(z1 / -z3, z2 / -z3, T(-1.0));
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

}  // namespace

/* ************************************************************************** */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> FitConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts) {
  // Build the data matrix using the provided (x,y) points.
  const int N = pts.cols();
  Eigen::Matrix<T, Eigen::Dynamic, 3> data =
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(N, 3);
  data.block(0, 0, N, 2) = pts.cwiseProduct(pts).transpose();
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
  const T lambda_ell = alpha11 + sqrt(alpha12 * alpha21);
  const T lambda_hyp = alpha11 - sqrt(alpha12 * alpha21);
  // And bundle everything together.
  const ConicFit<T> ell = BuildConicFit(D, lambda_ell, alpha11, alpha21);
  const ConicFit<T> hyp = BuildConicFit(D, lambda_hyp, alpha11, alpha21);

  return {ell, hyp};
}

/* ************************************************************************** */
template <typename T>
ConicFit<T> FitBestConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts) {
  //const auto [ellFit, hypFit] = FitConicCanonical<T>(pts);
  ConicFit<T> ellFit;
  ConicFit<T> hypFit;
  std::tie(ellFit, hypFit) = FitConicCanonical<T>(pts);
  return ((ellFit.residual_square < hypFit.residual_square) ? ellFit : hypFit);
}

/* ************************************************************************** */
template <typename T>
std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>> GetBestConicFits(
    const Eigen::Matrix<T, 3, 3>& R_EG,
    const Eigen::Matrix<T, 3, Eigen::Dynamic>& G_omegaB) {
  const size_t N = G_omegaB.cols();
  // Rotate data to proposed elliptical frame E.
  Eigen::Matrix<T, 3, Eigen::Dynamic> omegaB_E = R_EG * G_omegaB;

  // Splice the data by planes.
  Eigen::Matrix<T, 2, Eigen::Dynamic> xyData(2, N), xzData(2, N), yzData(2, N);
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
  //
  // std::cout << "XY fit type: " << pao::ConicTypeString(xyFit.c.Geo().type) << std::endl;
  // std::cout << "XZ fit type: " << pao::ConicTypeString(xzFit.c.Geo().type) << std::endl;
  // std::cout << "YZ fit type: " << pao::ConicTypeString(yzFit.c.Geo().type) << std::endl;

  return std::make_tuple(xyFit, xzFit, yzFit);
}

}  // namespace pao

#endif  // POLHODE_ALIGNMENT_CONIC_FITTING_TPP_
