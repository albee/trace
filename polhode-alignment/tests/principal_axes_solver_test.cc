/**
 * @file principal_axes_solver_test.cpp
 * @brief Unit tests for principal axes solver.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */
#include "polhode_alignment/principal_axes_solver.h"

#include <fstream>
#include <vector>
#include <string>

#include "gtest/gtest.h"
#include "polhode_alignment/geometry_utils.h"
#include "polhode_alignment/rigid_body_rotation.h"
#include "sph-vertigo/PrincipalAxesOpt.h"

using PlaneFit = std::pair<pao::ConicFit<double>, int>;

namespace {

namespace {

  template<typename M>
  M load_csv (const std::string & path) {
      std::ifstream indata;
      indata.open(path);
      std::string line;
      std::vector<double> values;
      uint rows = 0;
      while (std::getline(indata, line)) {
          std::stringstream lineStream(line);
          std::string cell;
          while (std::getline(lineStream, cell, ',')) {
              values.push_back(std::stod(cell));
          }
          ++rows;
      }
      return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
  }


// Returns the times vector and the omegas matrix.
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CreateCleanPolhode(size_t N) {
  // Create rigid body to generate data for testing.
  pao::RigidBodyParams params;
  params.J = Eigen::Vector3d{1.239, 1.1905, 1.0}.asDiagonal();  // PAOERES
  params.R_WB0 = Eigen::Matrix3d::Identity();   // Initial orientation.
  params.init = pao::DynamicsInitType::OMEGA0;  // Initialize with omega0;
  params.omega0 = Eigen::Vector3d{0, 0.939392242898362, 0.500486277097766};
  pao::RigidBodyRotation rbr{params};  // HE case.

  Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(N, 0.0, 4.0 * rbr.T());
  return std::tuple<Eigen::VectorXd, Eigen::MatrixXd> {times, rbr.PredictOmega(times)};
  //return {times, rbr.PredictOmega(times)};
}

void WriteDataToDisk(size_t N, const Eigen::VectorXd& times,
                     const Eigen::MatrixXd& Bc_omega_tru,
                     const Eigen::MatrixXd& Bn_omega_tru,
                     const Eigen::MatrixXd& Gn_omega, const double rads,
                     const Eigen::Vector3d& axis,
                     const Eigen::AngleAxisd& aa_BG_est) {
  const size_t cols = 1 + 3 + 3 + 3;
  Eigen::MatrixXd data = Eigen::MatrixXd::Zero(N, cols);
  data.block(0, 0, N, 1) = times;
  data.block(0, 1, N, 3) = Bc_omega_tru.transpose();
  data.block(0, 4, N, 3) = Bn_omega_tru.transpose();
  data.block(0, 7, N, 3) = Gn_omega.transpose();

  std::string path = "/home/charles/td_ws/polhode/test_results/unittest.dat";
  std::ofstream ostream(path);
  ostream << data << std::endl;
  ostream.close();

  path = "/home/charles/td_ws/polhode/test_results/out.tru";
  ostream = std::ofstream(path);
  ostream << rads << " " << axis.transpose() << " "
          << aa_BG_est.angle() * aa_BG_est.axis().transpose() << std::endl;
  ostream.close();
}

}  // namespace

Eigen::Matrix3f q2dcm(const float x, const float y, const float z, const float w) {
  Eigen::Matrix3f rot;
  // clang-format off
  rot << 1 - 2*pow(y, 2) - 2*pow(z, 2), 2*x*y - 2*z*w, 2*x*z + 2*y*w,
         2*x*y + 2*z*w, 1 - 2*pow(x, 2) - 2*pow(z, 2), 2*y*z - 2*x*w,
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*pow(x, 2) - 2*pow(y, 2);
  // clang-format on
  return rot;

}

/*


std::vector<std::pair<pao::ConicFit<double>, int>> GetBestConicFitsConstrained(const Eigen::Matrix3d& R, const Eigen::MatrixXd& omegaB_G) {
  // Rotate data to proposed elliptical frame E.
  Eigen::MatrixXd omegaB_E = R * omegaB_G;

  int N = omegaB_G.cols();

  // Splice the data by planes.
  Eigen::Matrix<double, 2, Eigen::Dynamic> xyData;
  Eigen::Matrix<double, 2, Eigen::Dynamic> xzData;
  Eigen::Matrix<double, 2, Eigen::Dynamic> yzData;
  xyData.resize(2, N);
  xzData.resize(2, N);
  yzData.resize(2, N);
  //Eigen::MatrixXd xyData(2, N), xzData(2, N), yzData(2, N);
  xyData.block(0, 0, 1, N) = omegaB_E.block(0, 0, 1, N);  // x to x
  xyData.block(1, 0, 1, N) = omegaB_E.block(1, 0, 1, N);  // y to y
  xzData.block(0, 0, 1, N) = omegaB_E.block(0, 0, 1, N);  // x to x
  xzData.block(1, 0, 1, N) = omegaB_E.block(2, 0, 1, N);  // z to y
  yzData.block(0, 0, 1, N) = omegaB_E.block(1, 0, 1, N);  // y to x
  yzData.block(1, 0, 1, N) = omegaB_E.block(2, 0, 1, N);  // z to y

  // Get all best fits.
  std::vector<std::pair<pao::ConicFit<double>, int>> fits;
  std::pair<pao::ConicFit<double>, pao::ConicFit<double>> xyFitPair =
      pao::FitConicCanonical(xyData);
  fits.emplace_back(std::make_pair(xyFitPair.first, 1));   // Elliptic fit.
  fits.emplace_back(std::make_pair(xyFitPair.second, 1));  // Hyperbolic fit.
  std::pair<pao::ConicFit<double>, pao::ConicFit<double>> xzFitPair =
      pao::FitConicCanonical(xzData);
  fits.push_back(std::make_pair(xzFitPair.first, 2));   // Elliptic fit.
  fits.push_back(std::make_pair(xzFitPair.second, 2));  // Hyperbolic fit.
  std::pair<pao::ConicFit<double>, pao::ConicFit<double>> yzFitPair =
      pao::FitConicCanonical(yzData);
  fits.push_back(std::make_pair(yzFitPair.first, 3));   // Elliptic fit.
  fits.push_back(std::make_pair(yzFitPair.second, 3));  // Hyperbolic fit.

  // Sort by residual, from smallest to highest.
  std::sort(fits.begin(), fits.end(),
            [](const std::pair<pao::ConicFit<double>, int>& lhs,
               const std::pair<pao::ConicFit<double>, int>& rhs) {
              return lhs.first.residual_square < rhs.first.residual_square;
            });
  for (int i = 0; i < 6; i++) {
    std::cout << "Residual " << i << ": " << fits[i].first.residual_square << std::endl;
  }

  // Keep track of number of elliptical and hyperbolic fits.
  int numEll = 0, numHyp = 0, planeIdx = 0;
  Eigen::Vector3i planesChosen{0, 0, 0};
  std::vector<std::pair<pao::ConicFit<double>, int>> fitsChosen;

  // Loop over six options.
  for (int i = 0; i < 6; i++) {
    if ((fits[i].first.c.Geo().type == pao::ConicType::kEllipse && numEll < 2 &&
         !pao::ValueInVec(fits[i].second, planesChosen)) ||
        (fits[i].first.c.Geo().type == pao::ConicType::kHyperbola && numHyp < 1 &&
         !pao::ValueInVec(fits[i].second, planesChosen))) {
      planesChosen(planeIdx) = fits[i].second;  // Record the plane.
      fitsChosen.push_back(fits[i]);            // Record the conic fit.
      std::cout << "Conic type: " << pao::ConicTypeString(fits[i].first.c.Geo().type) << std::endl;
      std::cout << "Conic: " << std::endl << fits[i].first.c << std::endl;
      planeIdx++;
    }
  }

  return fitsChosen;
}


std::tuple<Eigen::Vector3d, Eigen::Vector3d> GetHypAxes(
    const Eigen::Matrix3d& R_EG, const std::vector<PlaneFit>& fits, const Eigen::MatrixXd& omegaB_G) {
  // Rotate all data from geometric to elliptical frame E.
  Eigen::MatrixXd omegaB_E = R_EG * omegaB_G;

  int N = omegaB_G.cols();

  const Eigen::MatrixXi planeAxes_ =
      (Eigen::MatrixXi(2, 3) << 1, 1, 2, 2, 3, 3).finished();

  // Find plane with the best hyperbolic fit, and non-hyperbolic with low ecc.
  int hypPlane = 0, idx = 0;
  std::vector<int> circPlanes, circPlaneInd;
  std::vector<double> eccs;
  pao::Conic<double> hypK(1, 0, 3, 0, 0, -1);  // Dummy numbers.
  for (const PlaneFit& fit : fits) {
    if (fit.first.c.Geo().type == pao::ConicType::kHyperbola) {
      hypPlane = fit.second;
      hypK = fit.first.c;
    } else if (fit.first.c.Geo().type == pao::ConicType::kEllipse &&
               fit.first.c.Geo().ecc <= 0.35) {
      circPlanes.push_back(fit.second);
      circPlaneInd.push_back(idx);
      std::cout << "Ecc.: " << fit.first.c.Geo().ecc << std::endl;
    }
    eccs.push_back(fit.first.c.Geo().ecc);
    std::cout << "All Ecc's.: " << eccs[idx] << std::endl;
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
  Eigen::MatrixXd omegaB_Ehyp = Eigen::MatrixXd::Zero(2, N);
  omegaB_Ehyp.block(0, 0, 1, N) = omegaB_E.block(xHypProj, 0, 1, N);  // x
  omegaB_Ehyp.block(1, 0, 1, N) = omegaB_E.block(yHypProj, 0, 1, N);  // y

  // Assign each angular velocity in frame E to a quadrant (Q1, Q2, Q3, Q4), and
  // get the quadrant in which the dn-axis is located.
  Eigen::VectorXd n1dot = n1.transpose() * omegaB_Ehyp;
  Eigen::VectorXd n2dot = n2.transpose() * omegaB_Ehyp;

  int numInQuad1 = 0, numInQuad2 = 0, numInQuad3 = 0, numInQuad4 = 0;
  for (int i = 0; i < N; i++) {
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
  cnHat_E = pao::sgn(dotprod) * cnHat_E;

  return std::make_tuple(cnHat_E, dnHat_E);
}

Eigen::Matrix3d FindBodyFrame(const Eigen::Matrix3d& R_EG, const Eigen::MatrixXd& omegaB_G) {
  // Get the best constrained fits.
  std::vector<std::pair<pao::ConicFit<double>, int>> fits = GetBestConicFitsConstrained(R_EG, omegaB_G);

  int N = omegaB_G.cols();

  // Assing axes from R_EG to R_GB.
  Eigen::Vector3d cnHat_E, dnHat_E;
  std::tie(cnHat_E, dnHat_E) = GetHypAxes(R_EG, fits, omegaB_G);

  // Get the unit vectors for the cn, and dn axes in the G frame.
  // NOTE: I keep the transposed version of Tim's R_GE matrix.
  Eigen::Matrix3d R_GE = R_EG.transpose();
  Eigen::Vector3d cnHat_G = R_GE * cnHat_E;
  Eigen::Vector3d dnHat_G = R_GE * dnHat_E;

  // Get the average projection of omega(i) x omega(i+1) on the dn-axis.
  double muProjDn = 0;
  for (int i = 0; i < N - 1; i++) {
    muProjDn +=
        (1.0 / static_cast<double>(N - 1)) *
        dnHat_G.dot(pao::SkewMat<double>(omegaB_G.col(i)) * omegaB_G.col(i + 1));
  }

  // If projection follows right hand rule on average, it is LE; otherwise HE.
  // Specify R_BG using appropriate LE/HE elliptic functions relationships. The
  // sn-axis = cn_E x dn_E (LE) or dn_E x cn_E (HE).
  Eigen::Matrix3d R_GB;
  if (muProjDn >= 0) {
    //energy_ = EnergyState::LE;
    Eigen::Vector3d snHat_E = pao::SkewMat<double>(cnHat_E) * dnHat_E;
    Eigen::Vector3d snHat_G = R_GE * snHat_E;
    R_GB.col(0) = dnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = cnHat_G;

  } else {
    //energy_ = EnergyState::HE;
    Eigen::Vector3d snHat_E = pao::SkewMat<double>(dnHat_E) * cnHat_E;
    Eigen::Vector3d snHat_G = R_GE * snHat_E;
    R_GB.col(0) = cnHat_G;
    R_GB.col(1) = snHat_G;
    R_GB.col(2) = dnHat_G;
  }

  return R_GB;
}
*/

TEST(PrincipalAxesSolverTest, Construction) {
  // Bc: body frame, clean; Bn: body frame, clean.
  /*
  const size_t N = 200;
  //const auto [times, Bc_omega_tru] = CreateCleanPolhode(N);
  Eigen::VectorXd times;
  Eigen::MatrixXd Bc_omega_tru;
  std::tie(times, Bc_omega_tru) = CreateCleanPolhode(N);

  const Eigen::MatrixXd Bn_omega_tru =
      Bc_omega_tru + pao::SampleNormalMatrix<double>(3, N, 0.0, 0.001);

  // Purposefully rotate the polhode and record ground truth data.
  const double degs = 20;
  const double rads = degs * M_PI / 180.0;
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 1.0, 1.0).normalized();
  const Eigen::AngleAxisd aa_GB(rads, axis);
  const Eigen::MatrixXd Gn_omega = Eigen::Matrix3d(aa_GB) * Bn_omega_tru;
  */
  std::string filename;
  //filename = "/home/charles/td_ws/freeflyer-shared-td/develop/data/slam_data/analysis/smooth_omega60.csv";
  //filename = "/home/charles/td_ws/freeflyer-shared-td/develop/data/slam_data/analysis/smooth_omega90.csv";
  //filename = "/home/charles/td_ws/freeflyer-shared-td/develop/data/slam_data/analysis/smooth_omega120.csv";
  //filename = "/home/charles/td_ws/freeflyer-shared-td/develop/data/slam_data/analysis/smooth_omega150.csv";
  std::string TD_PATH = "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td";
  filename = TD_PATH + "/develop/data/slam_data/analysis/smooth_omega_test6_flipped.csv";
  // filename = "/home/charles/td_ws/freeflyer-shared-td/develop/data/slam_data/analysis/smooth_omega_test6_flipped.csv";
  Eigen::MatrixXd Gn_omega = load_csv<Eigen::MatrixXd>(filename).transpose();

  // Initial conditions.
  const Eigen::Vector3d theta_BG_init(0, 0, 0);
  Eigen::Vector3d theta_BG_est = theta_BG_init;
  Eigen::Quaterniond q_BG_est{Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())};

  // Now try to optimize.
  // auto* cost_function = pao::PrincipalAxesAlignmentAACost::Create(Gn_omega);
  auto* cost_function = pao::PrincipalAxesAlignmentCost::Create(Gn_omega);
  ceres::Problem problem;
  // problem.AddResidualBlock(cost_function, nullptr, theta_GB_est.data());
  problem.AddResidualBlock(cost_function, nullptr, q_BG_est.coeffs().data());
  problem.SetParameterization(q_BG_est.coeffs().data(),
                              new ceres::EigenQuaternionParameterization);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << summary.FullReport() << "\n";

  const Eigen::AngleAxisd aa_BG_est(q_BG_est);
  std::cout << "q  est: " << q_BG_est.coeffs().transpose() << std::endl;
  std::cout << "aa est: " << aa_BG_est.angle() * aa_BG_est.axis().transpose()
            << "\n";

  Eigen::Vector4f q_GP;
  q_GP << (float)q_BG_est.x(), (float)q_BG_est.y(), (float)q_BG_est.z(), (float)q_BG_est.w();
  Eigen::Matrix3f R_GP = q2dcm(q_GP(0), q_GP(1), q_GP(2), q_GP(3));

  Eigen::Matrix3f R_TP;
  R_TP << -0.019026452224050, 0.005224221963522, -0.999805331862479,
                  0.077059267410756, 0.997019487094011, 0.003743215317479,
                          0.996844954555062, -0.076973046319288, -0.019372318354941;

  Eigen::Matrix3f R_GE;
  sph::PrincipalAxesOpt<double> principal_axes_object(Gn_omega.cast<double>());
  R_GE = principal_axes_object.FindBodyFrame(R_GP.cast<double>().transpose()).cast<float>();
  Eigen::Matrix3f R_GT_;
  R_GT_ = R_GE * R_TP.transpose();

  //R_GT_ = R_GE * R_TP;

  //Eigen::Matrix3f R_GT_;
  //R_GT_ = R_GE;
  //sph::PrincipalAxesOpt<double> principal_axes_object(Gn_omega.cast<double>());
  //R_GT_ = principal_axes_object.FindBodyFrame(R_GE.cast<double>()).cast<float>();
  //R_GT_ = FindBodyFrame(R_GE.transpose().cast<double>(), Gn_omega.cast<double>()).cast<float>();
  //R_GT_ = FindBodyFrame(R_GE.cast<double>(), Gn_omega.cast<double>()).cast<float>();

  /*Eigen::Matrix3f R_x;
  R_x << 1, 0, 0,
          0, 0, -1,
          0, 1, 0;
  Eigen::Matrix3f R_y;
  R_y << 0, 0, 1,
          0, 1, 0,
          -1, 0, 0;

  Eigen::Matrix3f R_GT_ = R_GP * R_TP.transpose() * R_x.transpose() * R_y.transpose();
  */
  std::cout << "R_GP_est" << std::endl << R_GP << std::endl;
  std::cout << "R_GE_est" << std::endl << R_GE << std::endl;
  std::cout << "R_GT_est" << std::endl << R_GT_ << std::endl;


  //WriteDataToDisk(N, times, Bc_omega_tru, Bn_omega_tru, Gn_omega, rads, axis,
  //                aa_BG_est);
}

}  // namespace
