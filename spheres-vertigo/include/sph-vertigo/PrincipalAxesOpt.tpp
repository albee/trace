/**
 * @file PrincipalAxesOpt.tpp
 * @brief Object to estimate principal axes of rigid body.
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2021 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_PRINCIPALAXESOPT_TPP_
#define SPH_VERTIGO_PRINCIPALAXESOPT_TPP_

namespace sph {

/* ************************************************************************** */
template <typename T>
PrincipalAxesOpt<T>::PrincipalAxesOpt(const Matrix3X<T>& omegaB_G)
    : omegaB_G_(omegaB_G), N_(omegaB_G_.cols()) {}

/* ************************************************************************** */
template <typename T>
Matrix3<T> PrincipalAxesOpt<T>::FindBodyFrame(const Matrix3<T>& R_EG) {
  // Get the best constrained fits.
  fits_ = GetBestConicFitsConstrained(R_EG);

  // Assing axes from R_EG to R_GB.
  Vector3<T> cnHat_E, dnHat_E;
  std::tie(cnHat_E, dnHat_E) = GetHypAxes(R_EG, fits_);

  // Get the unit vectors for the cn, and dn axes in the G frame.
  // NOTE: I keep the transposed version of Tim's R_GE matrix.
  Matrix3<T> R_GE = R_EG.transpose();
  Vector3<T> cnHat_G = R_GE * cnHat_E;
  Vector3<T> dnHat_G = R_GE * dnHat_E;

  // Get the average projection of omega(i) x omega(i+1) on the dn-axis.
  T muProjDn = 0;
  for (size_t i = 0; i < N_ - 1; i++) {
    muProjDn +=
        (1.0 / static_cast<T>(N_ - 1)) *
        dnHat_G.dot(pao::SkewMat<T>(omegaB_G_.col(i)) * omegaB_G_.col(i + 1));
  }

  // If projection follows right hand rule on average, it is LE; otherwise HE.
  // Specify R_BG using appropriate LE/HE elliptic functions relationships. The
  // sn-axis = cn_E x dn_E (LE) or dn_E x cn_E (HE).
  Matrix3<T> R_GB;
  if (muProjDn >= 0) {
    energy_ = EnergyState::LE;
    Vector3<T> snHat_E = pao::SkewMat<T>(cnHat_E) * dnHat_E;
    Vector3<T> snHat_G = R_GE * snHat_E;
    R_GB.col(0) = dnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = cnHat_G;

  } else {
    energy_ = EnergyState::HE;
    Vector3<T> snHat_E = pao::SkewMat<T>(dnHat_E) * cnHat_E;
    Vector3<T> snHat_G = R_GE * snHat_E;
    R_GB.col(0) = cnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = dnHat_G;
  }

  return R_GB;
}

/* ************************************************************************** */
template <typename T>
std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>>
PrincipalAxesOpt<T>::GetBestConicFits(const Matrix3<T>& R) const {
  // Rotate data to proposed elliptical frame E.
  Matrix3X<T> omegaB_E = R * omegaB_G_;

  // Splice the data by planes.
  Matrix2X<T> xyData(2, N_), xzData(2, N_), yzData(2, N_);
  xyData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xyData.block(1, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to y
  xzData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y
  yzData.block(0, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to x
  yzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y

  // Get best fits in each plane.
  ConicFit<T> xyFit = FitBestConicCanonical<T>(xyData);
  ConicFit<T> xzFit = FitBestConicCanonical<T>(xzData);
  ConicFit<T> yzFit = FitBestConicCanonical<T>(yzData);

  return std::make_tuple(xyFit, xzFit, yzFit);
}

/* ************************************************************************** */
template <typename T>
std::vector<std::pair<ConicFit<T>, int>>
PrincipalAxesOpt<T>::GetBestConicFitsConstrained(const Matrix3<T>& R) const {
  // Rotate data to proposed elliptical frame E.
  Matrix3X<T> omegaB_E = R * omegaB_G_;

  // Splice the data by planes.
  Matrix2X<T> xyData(2, N_), xzData(2, N_), yzData(2, N_);
  xyData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xyData.block(1, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to y
  xzData.block(0, 0, 1, N_) = omegaB_E.block(0, 0, 1, N_);  // x to x
  xzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y
  yzData.block(0, 0, 1, N_) = omegaB_E.block(1, 0, 1, N_);  // y to x
  yzData.block(1, 0, 1, N_) = omegaB_E.block(2, 0, 1, N_);  // z to y

  // Get all best fits.
  std::vector<std::pair<ConicFit<T>, int>> fits;
  std::pair<ConicFit<T>, ConicFit<T>> xyFitPair = FitConicCanonical<T>(xyData);
  fits.emplace_back(std::make_pair(xyFitPair.first, 1));   // Elliptic fit.
  fits.emplace_back(std::make_pair(xyFitPair.second, 1));  // Hyperbolic fit.
  std::pair<ConicFit<T>, ConicFit<T>> xzFitPair = FitConicCanonical<T>(xzData);
  fits.push_back(std::make_pair(xzFitPair.first, 2));   // Elliptic fit.
  fits.push_back(std::make_pair(xzFitPair.second, 2));  // Hyperbolic fit.
  std::pair<ConicFit<T>, ConicFit<T>> yzFitPair = FitConicCanonical<T>(yzData);
  fits.push_back(std::make_pair(yzFitPair.first, 3));   // Elliptic fit.
  fits.push_back(std::make_pair(yzFitPair.second, 3));  // Hyperbolic fit.

  // Sort by residual, from smallest to highest.
  std::sort(fits.begin(), fits.end(),
            [](const std::pair<ConicFit<T>, int>& lhs,
               const std::pair<ConicFit<T>, int>& rhs) {
              return lhs.first.residual < rhs.first.residual;
            });

  // Keep track of number of elliptical and hyperbolic fits.
  int numEll = 0, numHyp = 0, planeIdx = 0;
  Eigen::Vector3i planesChosen{0, 0, 0};
  std::vector<std::pair<ConicFit<T>, int>> fitsChosen;

  // Loop over six options.
  for (int i = 0; i < 6; i++) {
    if ((fits[i].first.c.Geo().type == ConicType::ELLIPSE && numEll < 2 &&
         !ValueInVec(fits[i].second, planesChosen)) ||
        (fits[i].first.c.Geo().type == ConicType::HYPERBOLA && numHyp < 1 &&
         !ValueInVec(fits[i].second, planesChosen))) {
      planesChosen(planeIdx) = fits[i].second;  // Record the plane.
      fitsChosen.push_back(fits[i]);            // Record the conic fit.

      if (fits[i].first.c.Geo().type == ConicType::ELLIPSE) {
        numEll++;
      }
      else {
        numHyp++;
      }

      planeIdx++;
    }
    std::cout << "Conic fit: " << fits[i].first.c << std::endl;
    std::cout << "Residual: " << fits[i].first.residual << std::endl;
    std::cout << std::endl << std::endl << std::endl;
  }

  return fitsChosen;
}

/* ************************************************************************** */
template <typename T>
std::tuple<Vector3<T>, Vector3<T>> PrincipalAxesOpt<T>::GetHypAxes(
    const Matrix3<T>& R_EG, const std::vector<PlaneFit<T>>& fits) const {
  // Rotate all data from geometric to elliptical frame E.
  Matrix3X<T> omegaB_E = R_EG * omegaB_G_;

  // Find plane with the best hyperbolic fit, and non-hyperbolic with low ecc.
  int hypPlane = 0, hypPlaneIdx = -1, idx = 0;
  std::vector<int> circPlanes, circPlaneInd;
  std::vector<T> eccs;
  Conic<T> hypK(T(1), T(0), T(3), T(0), T(0), T(-1));  // Dummy numbers.
  for (const PlaneFit<T>& fit : fits) {
    if (fit.first.c.Geo().type == ConicType::HYPERBOLA) {
      hypPlane = fit.second;
      hypPlaneIdx = idx;
      hypK = fit.first.c;
    } else if (fit.first.c.Geo().type == ConicType::ELLIPSE &&
               fit.first.c.Geo().ecc <= T(0.35)) {
      circPlanes.push_back(fit.second);
      circPlaneInd.push_back(idx);
    }
    eccs.push_back(fit.first.c.Geo().ecc);
    idx++;
  }

  // NOTE: Only supporting tri-axial.
  // symmetry_ = InertiaSymmetry::TA;

  // Get the normal vectors of asymptotes to the hyperbolic fit.
  Vector2<T> n1{-std::sqrt(-hypK.A() / hypK.C()), T(1.0)};
  Vector2<T> n2{std::sqrt(-hypK.A() / hypK.C()), T(1.0)};

  // Get the projected omegas to the hyperbolic plane ((x,y) meas form).
  int xHypProj = planeAxes_(0, hypPlane - 1) - 1,  // Planes: 1:x, 2:y, 3:z,
      yHypProj = planeAxes_(1, hypPlane - 1) - 1;  // use `hypPlane` as col idx.
  Matrix2X<T> omegaB_Ehyp = MatrixX<T>::Zero(2, N_);
  omegaB_Ehyp.block(0, 0, 1, N_) = omegaB_E.block(xHypProj, 0, 1, N_);  // x
  omegaB_Ehyp.block(1, 0, 1, N_) = omegaB_E.block(yHypProj, 0, 1, N_);  // y

  // Assign each angular velocity in frame E to a quadrant (Q1, Q2, Q3, Q4), and
  // get the quadrant in which the dn-axis is located.
  VectorX<T> n1dot = n1.transpose() * omegaB_Ehyp;
  VectorX<T> n2dot = n2.transpose() * omegaB_Ehyp;

  int numInQuad1 = 0, numInQuad2 = 0, numInQuad3 = 0, numInQuad4 = 0;
  for (size_t i = 0; i < N_; i++) {
    T n1cur = n1dot(i), n2cur = n2dot(i);
    if (n1cur < T(0) && n2cur > T(0)) {
      numInQuad1++;
    } else if (n1cur > T(0) && n2cur > T(0)) {
      numInQuad2++;
    } else if (n1cur > T(0) && n2cur < T(0)) {
      numInQuad3++;
    } else if (n1cur < T(0) && n2cur < T(0)) {
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
  Vector3<T> dnHat_E{T(0.0), T(0.0), T(0.0)}, cnHat_E{T(0.0), T(0.0), T(0.0)};
  Eigen::Vector3i inds{1, 2, 3};
  if (maxQuad == 1) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(0, hypPlane - 1)) dnHat_E(i) = T(1.0);
      if (inds(i) == planeAxes_(1, hypPlane - 1)) cnHat_E(i) = T(1.0);
    }

  } else if (maxQuad == 2) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(1, hypPlane - 1)) dnHat_E(i) = T(1.0);
      if (inds(i) == planeAxes_(0, hypPlane - 1)) cnHat_E(i) = T(1.0);
    }

  } else if (maxQuad == 3) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(0, hypPlane - 1)) dnHat_E(i) = T(-1.0);
      if (inds(i) == planeAxes_(1, hypPlane - 1)) cnHat_E(i) = T(1.0);
    }

  } else if (maxQuad == 4) {
    for (int i = 0; i < 3; i++) {
      if (inds(i) == planeAxes_(1, hypPlane - 1)) dnHat_E(i) = T(-1.0);
      if (inds(i) == planeAxes_(0, hypPlane - 1)) cnHat_E(i) = T(1.0);
    }
  }

  // Make cn-axis sign so as to make the initial omega along the cn-axis +ve.
  T dotprod = omegaB_E.col(0).dot(cnHat_E);
  cnHat_E = sgn(dotprod) * cnHat_E;

  return std::make_tuple(cnHat_E, dnHat_E);
}

}  // namespace sph

#endif  // SPH_VERTIGO_PRINCIPALAXESOPT_TPP_
