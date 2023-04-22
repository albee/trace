/**
 * @file PrincipalAxesOpt.cpp
 * @brief Object to estimate principal axes of rigid body.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/PrincipalAxesOpt.h"

namespace sph {

/* ************************************************************************** */
PrincipalAxesOpt::PrincipalAxesOpt(const Eigen::MatrixXd& omegaB_G)
    : omegaB_G_(omegaB_G), N_(omegaB_G_.cols()) {}

/* ************************************************************************** */
Eigen::Matrix3d PrincipalAxesOpt::FindBodyFrame(const Eigen::Matrix3d& R_EG) {
  // Get the best constrained fits.
  fits_ = GetBestConicFitsConstrained(R_EG);

  // Assing axes from R_EG to R_GB.
  Eigen::Vector3d cnHat_E, dnHat_E;
  std::tie(cnHat_E, dnHat_E) = GetHypAxes(R_EG, fits_);

  // Get the unit vectors for the cn, and dn axes in the G frame.
  // NOTE: I keep the transposed version of Tim's R_GE matrix.
  Eigen::Matrix3d R_GE = R_EG.transpose();
  Eigen::Vector3d cnHat_G = R_GE * cnHat_E;
  Eigen::Vector3d dnHat_G = R_GE * dnHat_E;

  // Get the average projection of omega(i) x omega(i+1) on the dn-axis.
  double muProjDn = 0;
  for (size_t i = 0; i < N_ - 1; i++) {
    muProjDn +=
        (1.0 / static_cast<double>(N_ - 1)) *
        dnHat_G.dot(box::Skew<double>(omegaB_G_.col(i)) * omegaB_G_.col(i + 1));
  }

  // If projection follows right hand rule on average, it is LE; otherwise HE.
  // Specify R_BG using appropriate LE/HE elliptic functions relationships. The
  // sn-axis = cn_E x dn_E (LE) or dn_E x cn_E (HE).
  Eigen::Matrix3d R_GB;
  if (muProjDn >= 0) {
    energy_ = EnergyState::LE;
    Eigen::Vector3d snHat_E = box::Skew<double>(cnHat_E) * dnHat_E;
    Eigen::Vector3d snHat_G = R_GE * snHat_E;
    R_GB.col(0) = dnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = cnHat_G;

  } else {
    energy_ = EnergyState::HE;
    Eigen::Vector3d snHat_E = box::Skew<double>(dnHat_E) * cnHat_E;
    Eigen::Vector3d snHat_G = R_GE * snHat_E;
    R_GB.col(0) = cnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = dnHat_G;
  }

  return R_GB;
}

/* ************************************************************************** */
std::tuple<ConicFit<double>, ConicFit<double>, ConicFit<double>>
PrincipalAxesOpt::GetBestConicFits(const Eigen::Matrix3d& R) const {
  // Rotate data to proposed elliptical frame E.
  Eigen::MatrixXd omegaB_E = R * omegaB_G_;

  // Splice the data by planes.
  Eigen::MatrixXd xyData(3, N_), xzData(3, N_), yzData(3, N_);
  xyData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xyData.block(1, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to y
  xzData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y
  yzData.block(0, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to x
  yzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y

  // Get best fits in each plane.
  ConicFit<double> xyFit = FitBestConicCanonical<double>(xyData);
  ConicFit<double> xzFit = FitBestConicCanonical<double>(xzData);
  ConicFit<double> yzFit = FitBestConicCanonical<double>(yzData);

  return std::make_tuple(xyFit, xzFit, yzFit);
}

/* ************************************************************************** */
std::vector<std::pair<ConicFit<double>, int>>
PrincipalAxesOpt::GetBestConicFitsConstrained(const Eigen::Matrix3d& R) const {
  // Rotate data to proposed elliptical frame E.
  Eigen::MatrixXd omegaB_E = R * omegaB_G_;

  // Splice the data by planes.
  Eigen::MatrixXd xyData(2, N_), xzData(2, N_), yzData(2, N_);
  xyData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xyData.block(1, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to y
  xzData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y
  yzData.block(0, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to x
  yzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y

  // Get all best fits.
  std::vector<std::pair<ConicFit<double>, int>> fits;
  std::pair<ConicFit<double>, ConicFit<double>> xyFitPair =
      FitConicCanonical(xyData);
  fits.emplace_back(std::make_pair(xyFitPair.first, 1));   // Elliptic fit.
  fits.emplace_back(std::make_pair(xyFitPair.second, 1));  // Hyperbolic fit.
  std::pair<ConicFit<double>, ConicFit<double>> xzFitPair =
      FitConicCanonical(xzData);
  fits.push_back(std::make_pair(xzFitPair.first, 2));   // Elliptic fit.
  fits.push_back(std::make_pair(xzFitPair.second, 2));  // Hyperbolic fit.
  std::pair<ConicFit<double>, ConicFit<double>> yzFitPair =
      FitConicCanonical(yzData);
  fits.push_back(std::make_pair(yzFitPair.first, 3));   // Elliptic fit.
  fits.push_back(std::make_pair(yzFitPair.second, 3));  // Hyperbolic fit.

  // Sort by residual, from smallest to highest.
  std::sort(fits.begin(), fits.end(),
            [](const std::pair<ConicFit<double>, int>& lhs,
               const std::pair<ConicFit<double>, int>& rhs) {
              return lhs.first.residual < rhs.first.residual;
            });

  // Keep track of number of elliptical and hyperbolic fits.
  int numEll = 0, numHyp = 0, planeIdx = 0;
  Eigen::Vector3i planesChosen{0, 0, 0};
  std::vector<std::pair<ConicFit<double>, int>> fitsChosen;

  // Loop over six options.
  for (int i = 0; i < 6; i++) {
    if ((fits[i].first.c.Geo().type == ConicType::ELLIPSE && numEll < 2 &&
         !ValueInVec(fits[i].second, planesChosen)) ||
        (fits[i].first.c.Geo().type == ConicType::HYPERBOLA && numHyp < 1 &&
         !ValueInVec(fits[i].second, planesChosen))) {
      planesChosen(planeIdx) = fits[i].second;  // Record the plane.
      fitsChosen.push_back(fits[i]);            // Record the conic fit.

      planeIdx++;
    }
  }

  return fitsChosen;
}

/* ************************************************************************** */
std::tuple<Eigen::Vector3d, Eigen::Vector3d> PrincipalAxesOpt::GetHypAxes(
    const Eigen::Matrix3d& R_EG, const std::vector<PlaneFit>& fits) const {
  // Rotate all data from geometric to elliptical frame E.
  Eigen::MatrixXd omegaB_E = R_EG * omegaB_G_;

  // Find plane with the best hyperbolic fit, and non-hyperbolic with low ecc.
  int hypPlane = 0, hypPlaneIdx = -1, idx = 0;
  std::vector<int> circPlanes, circPlaneInd;
  std::vector<double> eccs;
  Conic<double> hypK(1, 0, 3, 0, 0, -1);  // Dummy numbers.
  for (const PlaneFit& fit : fits) {
    if (fit.first.c.Geo().type == ConicType::HYPERBOLA) {
      hypPlane = fit.second;
      hypPlaneIdx = idx;
      hypK = fit.first.c;
    } else if (fit.first.c.Geo().type == ConicType::ELLIPSE &&
               fit.first.c.Geo().ecc <= 0.35) {
      circPlanes.push_back(fit.second);
      circPlaneInd.push_back(idx);
    }
    eccs.push_back(fit.first.c.Geo().ecc);
    idx++;
  }

  // NOTE: Only supporting tri-axial.
  // symmetry_ = InertiaSymmetry::TA;

  // Get the normal vectors of asymptotes to the hyperbolic fit.
  Eigen::Vector2d n1{-std::sqrt(-hypK.A() / hypK.C()), 1.0};
  Eigen::Vector2d n2{std::sqrt(-hypK.A() / hypK.C()), 1.0};

  // Get the projected omegas to the hyperbolic plane ((x,y) meas form).
  int xHypProj = planeAxes_(0, hypPlane - 1) - 1,  // Planes: 1:x, 2:y, 3:z,
      yHypProj = planeAxes_(1, hypPlane - 1) - 1;  // use `hypPlane` as col idx.
  Eigen::MatrixXd omegaB_Ehyp = Eigen::MatrixXd::Zero(2, N_);
  omegaB_Ehyp.block(0, 0, 1, N_) = omegaB_E.block(xHypProj, 0, 1, N_);  // x
  omegaB_Ehyp.block(1, 0, 1, N_) = omegaB_E.block(yHypProj, 0, 1, N_);  // y

  // Assign each angular velocity in frame E to a quadrant (Q1, Q2, Q3, Q4), and
  // get the quadrant in which the dn-axis is located.
  Eigen::VectorXd n1dot = n1.transpose() * omegaB_Ehyp;
  Eigen::VectorXd n2dot = n2.transpose() * omegaB_Ehyp;

  int numInQuad1 = 0, numInQuad2 = 0, numInQuad3 = 0, numInQuad4 = 0;
  for (size_t i = 0; i < N_; i++) {
    double n1cur = n1dot(i), n2cur = n2dot(i);
    if (n1cur < 0 && n2cur > 0) {
      numInQuad1++;
    } else if (n1cur > 0 && n2cur > 0) {
      numInQuad2++;
    } else if (n1cur > 0 && n2cur < 0) {
      numInQuad3++;
    } else if (n1cur < 0 && n2cur < 0) {
      numInQuad4++;
    }
  }
  std::vector<int> quadCounts{numInQuad1, numInQuad2, numInQuad3, numInQuad4};
  int maxQuad =
      std::distance(quadCounts.begin(),
                    std::max_element(quadCounts.begin(), quadCounts.end())) +
      1;  // Account for the 0-base.

  // Get the axis index of the dn-axis, and then determine the index of the sn
  // and cn-axes. Make the dn-axis sign so as to make all omegas +ve along the
  // dn-axis.
  Eigen::Vector3d dnHat_E{0.0, 0.0, 0.0}, cnHat_E{0.0, 0.0, 0.0};
  Eigen::Vector3i inds{1, 2, 3};
  if (maxQuad == 1) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(0, hypPlane - 1)) dnHat_E(i) = 1.0;
      if (inds(i) == planeAxes_(1, hypPlane - 1)) cnHat_E(i) = 1.0;
    }

  } else if (maxQuad == 2) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(1, hypPlane - 1)) dnHat_E(i) = 1.0;
      if (inds(i) == planeAxes_(0, hypPlane - 1)) cnHat_E(i) = 1.0;
    }

  } else if (maxQuad == 3) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(0, hypPlane - 1)) dnHat_E(i) = -1.0;
      if (inds(i) == planeAxes_(1, hypPlane - 1)) cnHat_E(i) = 1.0;
    }

  } else if (maxQuad == 4) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(1, hypPlane - 1)) dnHat_E(i) = -1.0;
      if (inds(i) == planeAxes_(0, hypPlane - 1)) cnHat_E(i) = 1.0;
    }
  }

  // Make cn-axis sign so as to make the initial omega along the cn-axis +ve.
  double dotprod = omegaB_E.col(0).dot(cnHat_E);
  cnHat_E = sgn(dotprod) * cnHat_E;

  return std::make_tuple(cnHat_E, dnHat_E);
}

}  // namespace sph
