/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file ConicProjectionFactor.cpp
 *  @author Tonio Teran
 **/

#include "sph-vertigo/ConicProjectionFactor.h"
#include <gtsam/inference/Symbol.h>

namespace gtsam {

/* ************************************************************************* */
static void check(const SharedNoiseModel& noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument(boost::str(
        boost::format("ConicProjectionFactor: NoiseModel has dimension "
                      "%1% instead of %2%.") %
        noiseModel->dim() % m));
}

/* ************************************************************************* */
void ConicProjectionFactor::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  std::cout << "ConicProjectionFactor" << std::endl;
  // To print our the involved pose symbols.
  NonlinearFactor::print(s, keyFormatter);
  Symbol P(R_PG_);
  std::cout << "Sought rotation: " << P.chr() << P.index() << std::endl;

  if (estimation_ == EstimationType::DIRECT) {
    size_t num_meas = pairs_.size();
    std::cout << "Number of measurements: " << num_meas << std::endl;
    for (size_t k = 0; k < num_meas; k++) {
      RotPair pair = pairs_[k];
      Symbol Ri(pair.first), Rj(pair.second);
      std::cout << " - " << Ri.chr() << Ri.index() << " to " << Rj.chr()
                << Rj.index() << ", deltaT: " << deltaTs_[k] << std::endl;
    }
  } else {
    size_t num_meas = tuples_.size();
    std::cout << "Number of measurements: " << num_meas << std::endl;
    for (size_t k = 0; k < num_meas; k++) {
      RotTuple tuple = tuples_[k];
      Symbol Gi(std::get<0>(tuple)), Gj(std::get<1>(tuple)),
          Bi(std::get<2>(tuple)), Bj(std::get<3>(tuple));
      std::cout << " - " << Gi.chr() << Gi.index() << "," << Bi.chr()
                << Bi.index() << " to " << Gj.chr() << Gj.index() << ", "
                << Bj.chr() << Bj.index() << ", deltaT: " << deltaTs_[k]
                << std::endl;
    }
  }

  std::cout << std::endl;
}

/* ************************************************************************** */
bool ConicProjectionFactor::equals(const ConicProjectionFactor& f,
                                   const double tol) const {
  if (R_PG_ != f.R_PG_) return false;

  bool arePairsEqual = true;
  if (estimation_ == EstimationType::DIRECT) {
    for (size_t i = 0; i < pairs_.size(); i++) {
      if (pairs_[i].first != f.pairs_[i].first ||
          pairs_[i].second != f.pairs_[i].second ||
          deltaTs_[i] != f.deltaTs_[i]) {
        return false;
      }
    }
  } else {
    for (size_t i = 0; i < tuples_.size(); i++) {
      if (std::get<0>(tuples_[i]) != std::get<0>(f.tuples_[i]) ||
          std::get<1>(tuples_[i]) != std::get<1>(f.tuples_[i]) ||
          std::get<2>(tuples_[i]) != std::get<2>(f.tuples_[i]) ||
          std::get<3>(tuples_[i]) != std::get<3>(f.tuples_[i]) ||
          deltaTs_[i] != f.deltaTs_[i]) {
        return false;
      }
    }
  }

  // Base check makes sure keys are also the same.
  return arePairsEqual && Base::equals(f, tol);
}

/* ************************************************************************** */
void ConicProjectionFactor::add(const Key& Ri, const Key& Rj,
                                const double deltaT) {
  // Add keys only if they haven't already been added.
  if (std::find(keys_.begin(), keys_.end(), Ri) == keys_.end())
    keys_.push_back(Ri);
  if (std::find(keys_.begin(), keys_.end(), Rj) == keys_.end())
    keys_.push_back(Rj);

  // Keep a copy of the rotation pair, along with its delta time.
  pairs_.push_back(std::make_pair(Ri, Rj));
  deltaTs_.push_back(deltaT);
}

/* ************************************************************************** */
void ConicProjectionFactor::add(const Key& Gi, const Key& Gj, const Key& Bi,
                                const Key& Bj, const double deltaT) {
  // Add keys only if they haven't already been added.
  if (std::find(keys_.begin(), keys_.end(), Gi) == keys_.end())
    keys_.push_back(Gi);
  if (std::find(keys_.begin(), keys_.end(), Gj) == keys_.end())
    keys_.push_back(Gj);
  if (std::find(keys_.begin(), keys_.end(), Bi) == keys_.end())
    keys_.push_back(Bi);
  if (std::find(keys_.begin(), keys_.end(), Bj) == keys_.end())
    keys_.push_back(Bj);

  // Keep a copy of the rotation tuple, along with its delta time.
  tuples_.push_back(std::make_tuple(Gi, Gj, Bi, Bj));
  deltaTs_.push_back(deltaT);
}

/* ************************************************************************** */
Vector3 ConicProjectionFactor::evaluateError(const Rot3& R_PG,
                                             const std::vector<Point3>& omegas,
                                             OptionalJacobian<3, 3> H1) const {
  /*
  std::cout << "Initial R_PG:" << std::endl;
  std::cout << " - Theta: " << Rot3::Logmap(R_PG).transpose() << std::endl;
  R_PG.print();
  std::cout << "This are the points being used." << std::endl;
  std::cout << "omegaB_G = [" << std::endl;
  for (const auto& w : omegas) {
    std::cout << w.x() << " " << w.y() << " " << w.z() << ";..." << std::endl;
  }
  std::cout << "];" << std::endl;
  */

  // Common variables for the Jabobian and Jacobian-less case.
  size_t N = omegas.size();
  Matrix ones = Matrix::Ones(N, 1);
  // Projected plane matrices.
  Matrix Dxy(N, 3), Dxz(N, 3), Dyz(N, 3);
  // Parsed results from generalized eigenvalue problem solution.
  ConicFitResult xyResult, xzResult, yzResult;

  if (H1) {
    // Create Jacobians for the chain rule.
    // Projection error wrt squared augmented data matrix.
    Matrix19 H_err_DTDxy, H_err_DTDxz, H_err_DTDyz;
    // Squared augmented data matrix wrt aug data matrix.
    Matrix H_DTDxy_Dxy, H_DTDxz_Dxz, H_DTDyz_Dyz;
    // Augmented data matrix for each projection wrt transposed rotated omegas.
    Matrix H_Dxy_WT = Matrix::Zero(3 * N, 3 * N),
           H_Dxz_WT = Matrix::Zero(3 * N, 3 * N),
           H_Dyz_WT = Matrix::Zero(3 * N, 3 * N);

    // Common Jacobians.
    Matrix H_WT_W;           // Transposed rotated omegas wrt rotated omegas.
    Matrix H_W_R(3 * N, 3);  // Rotated omegas with respect to rotation.

    // Rotate the omegas, W = R*W0.
    gtsam::Matrix W(3, N);
    for (size_t i = 0; i < N; i++) {
      Matrix33 H_R;  // Jacobian of rotated omega wrt to R.
      // W.col(i) = R_PG.rotate(omegas[i], H_R).vector();
      W.col(i) = R_PG.inverse().rotate(omegas[i], H_R).vector();
      H_W_R.block(3 * i, 0, 3, 3) = H_R;
    }

    // std::cout << "This are the rotated omegas:" << std::endl;
    // std::cout << W.transpose() << std::endl;

    // Transpose the rotated omegas, get Jacobian, and get diagonals.
    Matrix WT = W.transpose();
    H_WT_W = TransposeSelfJacobian(W.rows(), W.cols());
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag2WTx(2 * WT.col(0));
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag2WTy(2 * WT.col(1));
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag2WTz(2 * WT.col(2));

    // Square all entries (Hadamard).
    Matrix WT2 = WT.cwiseProduct(WT);

    // Project to XY.
    Dxy.block(0, 0, N, 1) = WT2.block(0, 0, N, 1);  // All omega_x elements.
    Dxy.block(0, 1, N, 1) = WT2.block(0, 1, N, 1);  // All omega_y elements.
    Dxy.block(0, 2, N, 1) = ones;                   // All ones.
    H_Dxy_WT.block(0, 0, N, N) = diag2WTx;          // Assemble Jacobian.
    H_Dxy_WT.block(N, N, N, N) = diag2WTy;

    // Project to XZ.
    Dxz.block(0, 0, N, 1) = WT2.block(0, 0, N, 1);  // All omega_x elements.
    Dxz.block(0, 1, N, 1) = WT2.block(0, 2, N, 1);  // All omega_z elements.
    Dxz.block(0, 2, N, 1) = ones;                   // All ones.
    H_Dxz_WT.block(0, 0, N, N) = diag2WTx;          // Assemble Jacobian.
    H_Dxz_WT.block(N, 2 * N, N, N) = diag2WTz;

    // Project to YZ.
    Dyz.block(0, 0, N, 1) = WT2.block(0, 1, N, 1);  // All omega_y elements.
    Dyz.block(0, 1, N, 1) = WT2.block(0, 2, N, 1);  // All omega_z elements.
    Dyz.block(0, 2, N, 1) = ones;                   // All ones.
    H_Dyz_WT.block(0, N, N, N) = diag2WTy;          // Assemble Jacobian.
    H_Dyz_WT.block(N, 2 * N, N, N) = diag2WTz;

    // Get the Jacobians of the squared projections used for eigenvalues.
    H_DTDxy_Dxy = SquareSelfJacobian(Dxy.col(0), Dxy.col(1), Dxy.col(2), N);
    H_DTDxz_Dxz = SquareSelfJacobian(Dxz.col(0), Dxz.col(1), Dxz.col(2), N);
    H_DTDyz_Dyz = SquareSelfJacobian(Dyz.col(0), Dyz.col(1), Dyz.col(2), N);

    // Solve generalized eigenvalue problem.
    ges_.compute(Dxy.transpose() * Dxy, C_);  // XY plane conic fit.
    xyResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());

    ges_.compute(Dxz.transpose() * Dxz, C_);  // XZ plane conic fit.
    xzResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());

    ges_.compute(Dyz.transpose() * Dyz, C_);  // YZ plane conic fit.
    yzResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());

    // Get the Jacobians using eigenvalue perturbation analysis.
    H_err_DTDxy = EigenvaluePerturbationJacobian(xyResult.v);
    H_err_DTDxz = EigenvaluePerturbationJacobian(xzResult.v);
    H_err_DTDyz = EigenvaluePerturbationJacobian(yzResult.v);

    // Chain-rule Jacobians to get error wrt each plane and then stack.
    Matrix H_errxy_WT = H_err_DTDxy * H_DTDxy_Dxy * H_Dxy_WT;  // (1 x 3N)
    Matrix H_errxz_WT = H_err_DTDxz * H_DTDxz_Dxz * H_Dxz_WT;  // (1 x 3N)
    Matrix H_erryz_WT = H_err_DTDyz * H_DTDyz_Dyz * H_Dyz_WT;  // (1 x 3N)

    Matrix H_err_WT(3, 3 * N);
    H_err_WT.row(0) = H_errxy_WT;
    H_err_WT.row(1) = H_errxz_WT;
    H_err_WT.row(2) = H_erryz_WT;

    *H1 = H_err_WT * H_WT_W * H_W_R;

  } else {  // No Jacobian calculation.
    // Rotate the omegas, W = R*W0.
    gtsam::Matrix W(3, N);
    for (size_t i = 0; i < N; i++) {
      // W.col(i) = R_PG.rotate(omegas[i]).vector();
      W.col(i) = R_PG.inverse().rotate(omegas[i]).vector();
    }

    // Transpose the rotated omegas, get Jacobian, and get diagonals.
    Matrix WT = W.transpose();

    // Square all entries (Hadamard).
    Matrix WT2 = WT.cwiseProduct(WT);

    // Project to XY.
    Dxy.block(0, 0, N, 1) = WT2.block(0, 0, N, 1);  // All omega_x elements.
    Dxy.block(0, 1, N, 1) = WT2.block(0, 1, N, 1);  // All omega_y elements.
    Dxy.block(0, 2, N, 1) = ones;                   // All ones.

    // Project to XZ.
    Dxz.block(0, 0, N, 1) = WT2.block(0, 0, N, 1);  // All omega_x elements.
    Dxz.block(0, 1, N, 1) = WT2.block(0, 2, N, 1);  // All omega_z elements.
    Dxz.block(0, 2, N, 1) = ones;                   // All ones.

    // Project to YZ.
    Dyz.block(0, 0, N, 1) = WT2.block(0, 1, N, 1);  // All omega_y elements.
    Dyz.block(0, 1, N, 1) = WT2.block(0, 2, N, 1);  // All omega_z elements.
    Dyz.block(0, 2, N, 1) = ones;                   // All ones.

    // Solve generalized eigenvalue problem.
    ges_.compute(Dxy.transpose() * Dxy, C_);  // XY plane conic fit.
    xyResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());
    ges_.compute(Dxz.transpose() * Dxz, C_);  // XZ plane conic fit.
    xzResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());
    ges_.compute(Dyz.transpose() * Dyz, C_);  // YZ plane conic fit.
    yzResult = ParseEigenPair(ges_.eigenvectors(), ges_.alphas(), ges_.betas());
  }

  /*
  std::cout << "RESULT for the XY plane:" << std::endl;
  std::cout << xyResult << std::endl;
  std::cout << "RESULT for the XZ plane:" << std::endl;
  std::cout << xzResult << std::endl;
  std::cout << "RESULT for the YZ plane:" << std::endl;
  std::cout << yzResult << std::endl;
  */

  // Get the full concatenated error vector.
  Vector3 errorVector(xyResult.cost, xzResult.cost, yzResult.cost);
  return errorVector;
}

/* **************************************************************************/
boost::shared_ptr<GaussianFactor> ConicProjectionFactor::linearize(
    const Values& vals) const {
  // Same as linearize in NoiseModelFactor

  // Only linearize if the factor is active
  if (!active(vals)) return boost::shared_ptr<JacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<Matrix> A(size());
  Vector b = -unwhitenedError(vals, A);
  check(noiseModel_, b.size());

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<Key, Matrix>> terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
}

/* **************************************************************************/
Vector ConicProjectionFactor::unwhitenedError(
    const Values& vals, boost::optional<std::vector<Matrix>&> H) const {
  size_t Nvars = size();

  if (this->active(vals)) {
    // Extract the angular velocity estimates to evaluate the error.
    std::vector<Point3> omegas = extractAngularVelocities(vals);

    if (H) {
      // Set Jacobians to zero for variables that are not the sought rotation.
      for (size_t k = 1; k < Nvars; k++) {
        if (type_ == VariableType::ROT) {
          (*H)[k] = Matrix::Zero(3, 3);
        } else {
          (*H)[k] = Matrix::Zero(3, 6);
        }
      }
      return evaluateError(vals.at<Rot3>(keys_[0]), omegas, (*H)[0]);
    } else {
      return evaluateError(vals.at<Rot3>(keys_[0]), omegas);
    }

  } else {
    return Vector::Zero(this->dim());
  }
}

/* ************************************************************************** */
double ConicProjectionFactor::error(const Values& vals) const {
  // Make sure that there are measurements inside as well.
  // TODO(tonioteran) Choosing dummy value of measurements available.
  if (active(vals) && (pairs_.size() > 5 || tuples_.size() > 5)) {
    const Vector b = unwhitenedError(vals);
    check(noiseModel_, b.size());
    return b.sum();
    // if (noiseModel_) {
    //   return 0.5 * noiseModel_->distance(b);
    // } else {
    //   return 0.5 * b.squaredNorm();
    // }
  } else {
    return 0.0;
  }
}

/* ************************************************************************** */
std::vector<Point3> ConicProjectionFactor::extractAngularVelocities(
    const Values& vals) const {
  std::vector<Point3> omegas;
  size_t k = 0;
  if (estimation_ == EstimationType::DIRECT) {
    // Loop over pairs of rotations to extract angular velocity estimate.
    for (const RotPair& p : pairs_) {
      Rot3 Ri, Rj;
      if (type_ == VariableType::ROT) {
        Ri = vals.at<Rot3>(p.first);
        Rj = vals.at<Rot3>(p.second);
      } else {
        Ri = vals.at<Pose3>(p.first).rotation();
        Rj = vals.at<Pose3>(p.second).rotation();
      }
      Rot3 Rij = Ri.inverse() * Rj;
      Vector3 thetaij = Rot3::Logmap(Rij);
      Point3 omega = (1 / deltaTs_[k]) * Point3(thetaij);

      omegas.push_back(omega);
      k++;
    }
  } else {
    // Loop over tuples of rotations to extract angular velocity estimate.
    for (const RotTuple& p : tuples_) {
      Rot3 Rij;
      if (type_ == VariableType::ROT) {
        Rot3 R_GiBi = vals.at<Rot3>(std::get<0>(p));
        Rot3 R_GjBj = vals.at<Rot3>(std::get<1>(p));
        Rot3 R_WBi = vals.at<Rot3>(std::get<2>(p));
        Rot3 R_WBj = vals.at<Rot3>(std::get<3>(p));

        Rij = R_GiBi * R_WBi.inverse() * R_WBj * R_GjBj.inverse();
      } else {
        Pose3 T_GiBi = vals.at<Pose3>(std::get<0>(p));
        Pose3 T_GjBj = vals.at<Pose3>(std::get<1>(p));
        Pose3 T_WBi = vals.at<Pose3>(std::get<2>(p));
        Pose3 T_WBj = vals.at<Pose3>(std::get<3>(p));

        Rij = (T_GiBi * T_WBi.inverse() * T_WBj * T_GjBj.inverse()).rotation();
      }
      Vector3 thetaij = Rot3::Logmap(Rij);
      Point3 omega = (1 / deltaTs_[k]) * Point3(thetaij);

      omegas.push_back(omega);
      k++;
    }
  }

  return omegas;
}

/* ************************************************************************** */
ConicProjectionFactor::ConicFitResult ConicProjectionFactor::ParseEigenPair(
    const EigenvectorsType& eigenvecs, const ComplexVectorType& alphas,
    const VectorType& betas) const {
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

  // Determine the conic with the lowest cost.
  double ell_residual = std::abs(ell_lambda);
  double hyp_residual = std::abs(hyp_lambda);
  size_t best_conic = (ell_residual < hyp_residual) ? ell_pos : hyp_pos;

  // Scale eigenvectors [a b f]^T so that f = -1 to comply with constraint.
  Vector3 raw_vec(eigenvecs(0, best_conic).real(),
                  eigenvecs(1, best_conic).real(),
                  eigenvecs(2, best_conic).real());
  Vector3 best_vec = -raw_vec / raw_vec(2);
  // Vector3 best_vec(
  //     {-eigenvecs(0, best_conic).real() / eigenvecs(2, best_conic).real(),
  //      -eigenvecs(1, best_conic).real() / eigenvecs(2, best_conic).real(),
  //      -eigenvecs(2, best_conic).real() / eigenvecs(2, best_conic).real()});

  // Build solution and return.
  ConicFitResult result;
  result.cost = (best_conic == ell_pos) ? ell_residual : hyp_residual;
  result.type =
      (best_conic == ell_pos) ? ConicType::ELLIPSE : ConicType::HYPERBOLA;
  result.K = best_vec;
  result.v = raw_vec;
  return result;
}

/* ************************************************************************** */
Matrix ConicProjectionFactor::TransposeSelfJacobian(const size_t m,
                                                    const size_t n) const {
  Matrix H_MT_M(m * n, m * n);

  // Each block is outer product between canonical vectors of length m and n.
  for (size_t row = 0; row < m; row++) {
    // Create the canonical unit vector of `m` size.
    Vector u_3 = CanonicalUnitVector(m, row);
    for (size_t col = 0; col < n; col++) {
      Vector u_m = CanonicalUnitVector(n, col);
      H_MT_M.block(row * n, col * m, n, m) = u_m * u_3.transpose();
    }
  }

  return H_MT_M;
}

/* ************************************************************************** */
Matrix ConicProjectionFactor::SquareSelfJacobian(const Vector& x1,
                                                 const Vector& x2,
                                                 const Vector& x3,
                                                 const size_t N) const {
  auto x1T = x1.transpose();
  auto x2T = x2.transpose();
  auto x3T = x3.transpose();
  Matrix H_MTM_M = Matrix::Zero(9, 3 * N);

  // Assign the first N columns.
  H_MTM_M.block(0, 0, 1, N) = 2 * x1T;
  H_MTM_M.block(1, 0, 1, N) = x2T;
  H_MTM_M.block(2, 0, 1, N) = x3T;
  H_MTM_M.block(3, 0, 1, N) = x2T;
  H_MTM_M.block(6, 0, 1, N) = x3T;

  // Assign the second block of N columns.
  H_MTM_M.block(1, N, 1, N) = x1T;
  H_MTM_M.block(3, N, 1, N) = x1T;
  H_MTM_M.block(4, N, 1, N) = 2 * x2T;
  H_MTM_M.block(5, N, 1, N) = x3T;
  H_MTM_M.block(7, N, 1, N) = x3T;

  // Assign the last and third block of N columns.
  H_MTM_M.block(2, 2 * N, 1, N) = x1T;
  H_MTM_M.block(5, 2 * N, 1, N) = x2T;
  H_MTM_M.block(6, 2 * N, 1, N) = x1T;
  H_MTM_M.block(7, 2 * N, 1, N) = x2T;
  H_MTM_M.block(8, 2 * N, 1, N) = 2 * x3T;

  return H_MTM_M;
}

/* ************************************************************************** */
Vector ConicProjectionFactor::CanonicalUnitVector(const size_t n,
                                                  const size_t i) const {
  Vector u = Vector::Zero(n);
  u(i) = 1.0;
  return u;
}

/* ************************************************************************** */
Matrix ConicProjectionFactor::EigenvaluePerturbationJacobian(
    const Vector3& eigenvector) const {
  Matrix H_err_DTD(1, 9);

  // Compute the six different values.
  double v0v0 = eigenvector(0) * eigenvector(0);
  double v0v1_2 = 2 * eigenvector(0) * eigenvector(1);
  double v0v2_2 = 2 * eigenvector(0) * eigenvector(2);
  double v1v1 = eigenvector(1) * eigenvector(1);
  double v1v2_2 = 2 * eigenvector(1) * eigenvector(2);
  double v2v2 = eigenvector(2) * eigenvector(2);

  // Build the row vector Jacobian.
  H_err_DTD << v0v0, v0v1_2, v0v2_2, v0v1_2, v1v1, v1v2_2, v0v2_2, v1v2_2, v2v2;
  return H_err_DTD;
}

}  // namespace gtsam
