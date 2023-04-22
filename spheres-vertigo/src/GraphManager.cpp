/**
 * @file GraphManager.cpp
 * @brief Class for dealing with inertial measurements.
 * @date August 01, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/GraphManager.h"

namespace sph {

void WritePerfFile(const int t, std::ofstream *outPerf, gtsam::ISAM2 *isam2,
                   gtsam::ISAM2 *isam2Fixed, gtsam::ISAM2 *isam2Custom,
                   const double time, const double timeFixed,
                   const double timeCustom, const gtsam::ISAM2Result &stats,
                   const gtsam::ISAM2Result &statsFixed,
                   const gtsam::ISAM2Result &statsCustom,
                   const gtsam::Values &result,
                   const gtsam::Values &resultFixed,
                   const gtsam::Values &resultCustom) {
  // Format:
  // step| trad [s]| smart [s]| dr [s]|
  (*outPerf) << t << " " << time << " " << timeFixed << " " << timeCustom
             << " ";
  // Observed keys: keys for variables that were observed, i.e., not unused.
  (*outPerf) << stats.observedKeys.size() << " "
             << statsFixed.observedKeys.size() << " "
             << statsCustom.observedKeys.size() << " ";
  // Unused keys: Unused keys, and indices for unused keys.
  (*outPerf) << stats.unusedKeys.size() << " " << statsFixed.unusedKeys.size()
             << " " << statsCustom.unusedKeys.size() << " ";
  // Variables relinearized: # of vars relinearized b/c of their linear
  // deltas.
  (*outPerf) << stats.getVariablesRelinearized() << " "
             << statsFixed.getVariablesRelinearized() << " "
             << statsCustom.getVariablesRelinearized() << " ";
  // Variables reeliminated: # vars reelimn due to new factors.
  (*outPerf) << stats.getVariablesReeliminated() << " "
             << statsFixed.getVariablesReeliminated() << " "
             << statsCustom.getVariablesReeliminated() << " ";
  // Factors recalculated: # factors included in reelimination of Bayes tree.
  (*outPerf) << stats.factorsRecalculated << " "
             << statsFixed.factorsRecalculated << " "
             << statsCustom.factorsRecalculated << " ";
  // Number of cliques in the tree.
  (*outPerf) << stats.getCliques() << " " << statsFixed.getCliques() << " "
             << statsCustom.getCliques() << " ";
  // Total number of variables.
  (*outPerf) << result.size() << " " << resultFixed.size() << " "
             << resultCustom.size() << " ";
  // Total number of factors.
  gtsam::NonlinearFactorGraph G = isam2->getFactorsUnsafe(),
                              GF = isam2Fixed->getFactorsUnsafe(),
                              GC = isam2Custom->getFactorsUnsafe();
  (*outPerf) << G.size() << " " << GF.size() << " " << GC.size() << " ";

  // Clique data.
  gtsam::BayesTreeCliqueData cliqData = isam2->getCliqueData();
  gtsam::BayesTreeCliqueData cliqDataF = isam2Fixed->getCliqueData();
  gtsam::BayesTreeCliqueData cliqDataC = isam2Custom->getCliqueData();
  gtsam::BayesTreeCliqueStats cliqStats = cliqData.getStats();
  gtsam::BayesTreeCliqueStats cliqStatsF = cliqDataF.getStats();
  gtsam::BayesTreeCliqueStats cliqStatsC = cliqDataC.getStats();
  // Avg conditional size.
  (*outPerf) << cliqStats.avgConditionalSize << " "
             << cliqStatsF.avgConditionalSize << " "
             << cliqStatsC.avgConditionalSize << " ";
  // Max conditional size.
  (*outPerf) << cliqStats.maxConditionalSize << " "
             << cliqStatsF.maxConditionalSize << " "
             << cliqStatsC.maxConditionalSize << " ";
  // Avg separator size.
  (*outPerf) << cliqStats.avgSeparatorSize << " " << cliqStatsF.avgSeparatorSize
             << " " << cliqStatsC.avgSeparatorSize << " ";
  // Avg separator size.
  (*outPerf) << cliqStats.maxSeparatorSize << " " << cliqStatsF.maxSeparatorSize
             << " " << cliqStatsC.maxSeparatorSize << " ";
  // Total conditionals.
  (*outPerf) << cliqData.conditionalSizes.size() << " "
             << cliqDataF.conditionalSizes.size() << " "
             << cliqDataC.conditionalSizes.size() << " ";
  // Total separators.
  (*outPerf) << cliqData.separatorSizes.size() << " "
             << cliqDataF.separatorSizes.size() << " "
             << cliqDataC.separatorSizes.size();

  (*outPerf) << std::endl;
}

/* ************************************************************************** */
GraphManager::GraphManager(std::shared_ptr<FrontEnd> fe,
                           const GraphParams &params)
    : fe_(fe), params_(params) {
  // Initialize preintegration.
  odometer_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      params_.pimParams, params_.bias0);

  // Initialize Bayes tree.
  isam_ = std::make_shared<gtsam::ISAM2>(params_.isam2Params);

  // Get the camera frame in body coordinates.
  T_BC_ = fe_->T_BC();

  // Instantiate constant noise models.
  // The noise model is *I believe* in R3, with the first two
  // dimensions indicating the bearing error. Given below are reasonable
  // standard deviations of a unit vector in the plane tangent to the unit ball
  // [-]. The final dimension is the range uncertainty [m].
  blobNoiseModel_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.1736, 0.1736, 0.2)
          .finished());  // sin(10deg), 0.2m

  // Create the output streams.
  pao_ = std::ofstream("/Users/tonio/Documents/MATLAB/matlab/dynamics/out.pao");
  perf_ =
      std::ofstream("/Users/tonio/Documents/MATLAB/matlab/dynamics/out.perf");
  tree_ =
      std::ofstream("/Users/tonio/Documents/MATLAB/matlab/dynamics/out.tree");
  g2o_ = std::ofstream("out.g2o");
  imu_ = std::ofstream("imu.g2o");
  combined_ = std::ofstream("combined.g2o");
  ratios_ =
      std::ofstream("/Users/tonio/Documents/MATLAB/matlab/dynamics/out.ratios");
  momentum_ = std::ofstream(
      "/Users/tonio/Documents/MATLAB/matlab/dynamics/out.momentum");
  jpar_ =
      std::ofstream("/Users/tonio/Documents/MATLAB/matlab/dynamics/out.jpar");

  // Setup the Conic Projection factor.
  conicFactor_ = boost::make_shared<gtsam::ConicProjectionFactor>(
      gtsam::Symbol('R', 0), gtsam::noiseModel::Isotropic::Sigma(3, 1.0),
      gtsam::ConicProjectionFactor::VariableType::POSE,
      gtsam::ConicProjectionFactor::EstimationType::INDIRECT);

  // Create blank image to avoid nullpointer for matched frame.
  matched_ = cv::Mat(1280, 480, CV_8UC3, cv::Scalar(0, 0, 0));
}

/* ************************************************************************** */
GraphManager::~GraphManager() {
  ExportAngularVelocities(
      "/Users/tonio/Documents/MATLAB/matlab/dynamics/out.omegasigma");
  pao_.close();
  perf_.close();
  tree_.close();
  g2o_.close();
  imu_.close();
  combined_.close();
  ratios_.close();
  momentum_.close();
  jpar_.close();
}

/* ************************************************************************** */
void GraphManager::AddImuMeasurement(const Eigen::Vector3d &accel,
                                     const Eigen::Vector3d &omega,
                                     const double dt) {
  if (!fgReady_) return;
  odometer_->integrateMeasurement(accel, omega, dt);
}

/* ************************************************************************** */
void GraphManager::AddGlobalMet(const MsgGlobalMet &gm) { gm_.push_back(gm); }

/* ************************************************************************** */
void GraphManager::AddImage(const sph::StereoFrame &frame) {
  std::cout << "(GraphManager::AddImage) entering:" << std::endl;  // TEMP
  std::cout << " Image num: " << frame.id << std::endl;
  // Extract information from frame.
  omsci::StereoFrame processed_frame;
  fe_->ProcessImage(frame, &processed_frame);
  std::cout << "--------" << std::endl;  // TEMP

  // Create the fist keyframe and initialize graph with priors.
  if (!fgReady_) {
    std::cout << "Initializing the estimator" << std::endl;

    if (processed_frame.leftRightMatches3D.size() < 7) {
      std::cout << "Cannont initialize. Insufficient triangulated features"
                << std::endl;
    } else {
      processed_frames_.push_back(processed_frame);
      raw_frames_.push_back(frame);
      times_.push_back(frame.timestamp);
      InitFactorGraph(frame);
      fgReady_ = true;
      std::cout << "Successfully initialized." << std::endl;
      std::cout << "++++++++++++++++++++++++." << std::endl << std::endl;
    }
    return;
  }

  // Get the relative pose measurement and inliers by matching stereo pairs.
  sph::Pose3D T_CiCj;
  omsci::StereoFrame inlier_frame_i, inlier_frame_j;
  fe_->MatchPair(processed_frames_[curStateId_ - 1], processed_frame,
                 &inlier_frame_i, &inlier_frame_j, &T_CiCj);

  // Get the latest matched frame.
  cv::Mat out_matched;
  fe_->DrawMatched(frame, processed_frame, inlier_frame_i, inlier_frame_j,
                   &out_matched);
  matched_ = out_matched.clone();

  // Only proceed if we have a valid relative pose measurement.
  if (T_CiCj.R.determinant() == -1 &&
      inlier_frame_j.leftRightMatches3D.size() < 6) {
    std::cout << "Invalid relative pose. Skipping." << std::endl;
    std::cout << "----BAD BAD BAD BAD BAD----." << std::endl
              << std::endl
              << std::endl;
    return;
  }

  // Create the geometric chain from visual odometry if it's the 1st pair.
  if (!geometricChainReady_) {
    std::cout << "First pair. Initializing geometric frame." << std::endl;

    Pose3D T_G0B0;
    fe_->GeometricFrame(inlier_frame_i, &T_G0B0);
    Pose3D T0(corresp_gm_[0].R, corresp_gm_[0].t);  // Init using global met.
    InitGeomChain(T_G0B0, T0);
    T_G0B0.cov = 1e-8 * Eigen::MatrixXd::Identity(6, 6);

    geometricChainReady_ = true;
  }

  // Proceed to add VO relative pose, IMU information, and kinematic constraint.
  std::cout << "Need to add new keyframe from img " << frame.id << std::endl
            << std::flush;
  std::cout << "Adding factors between x" << (curStateId_ - 1) << " and x"
            << curStateId_ << std::endl
            << std::flush;
  perf_ << frame.timestamp << " " << curStateId_ << " ";  // Initial.

  Pose3D T_BiBj = T_BC_ * T_CiCj * T_BC_.inverse();  // Relative pose meas.
  AddFactors(T_BiBj, frame);
  if (curStateId_ > 7)
    AttemptLoopClosure(processed_frame, frame);
  else
    perf_ << 0.0 << " ";

  // TEMP
  double deltaT = frame.timestamp - raw_frames_[curStateId_ - 1].timestamp;
  deltaT /= 1000.0;  // Get the time in seconds.
  std::cout << "Time difference between images: " << deltaT << std::endl;
  if (deltaT < 1.0) {
    std::cout << " + Small! Adding conic and inertia factors" << std::endl;

    gtsam::Symbol GiBi('G', curStateId_ - 1), GjBj('G', curStateId_);
    gtsam::Symbol WBi('T', curStateId_ - 1), WBj('T', curStateId_);
    gtsam::Symbol Jsym('J', 0), hij('h', curStateId_),
        hij_past('h', curStateId_ - 1);
    gtsam::Symbol bhij('R', curStateId_), bhij_past('R', curStateId_ - 1);

    // TODO(tonioteran) Hardcoding the actual delta time. Workarounds?
    conicFactor_->add(GiBi, GjBj, WBi, WBj, 0.5);
    // conicFactor_->print("Conic factor thus far...\n");

    // Compute the correct noise model here.
    // Compute the uncertainty as in Settefield PhD, Eq. A.13:
    gtsam::Pose3 T_GiBi = estimate_.at<gtsam::Pose3>(GiBi);
    gtsam::Matrix R_GiBi = T_GiBi.rotation().matrix();
    gtsam::Matrix R_GiBi_T = R_GiBi.transpose();
    gtsam::Matrix Sigma_R_GiBi =
        isam_->marginalCovariance(GiBi).block(0, 0, 3, 3);

    gtsam::Pose3 T_GjBj = estimate_.at<gtsam::Pose3>(GjBj);
    gtsam::Matrix Sigma_R_GjBj =
        isam_->marginalCovariance(GjBj).block(0, 0, 3, 3);

    gtsam::Pose3 T_WBi = estimate_.at<gtsam::Pose3>(WBi);
    gtsam::Matrix R_WBi = T_WBi.rotation().matrix();
    gtsam::Matrix Sigma_R_WBi =
        isam_->marginalCovariance(WBi).block(0, 0, 3, 3);

    gtsam::Pose3 T_WBj = estimate_.at<gtsam::Pose3>(WBj);
    gtsam::Matrix R_WBj = T_WBj.rotation().matrix();
    gtsam::Matrix Sigma_R_WBj =
        isam_->marginalCovariance(WBj).block(0, 0, 3, 3);

    gtsam::Matrix Sigma_omega =
        (1.0 / 0.5) *
        (R_GiBi * (Sigma_R_GiBi + Sigma_R_WBi) * R_GiBi_T +
         R_GiBi * R_WBi.transpose() * R_WBj * (Sigma_R_WBj + Sigma_R_GjBj) *
             R_WBj.transpose() * R_WBi * R_GiBi_T);

    // TEMP Initialize with ground truth for principal axes.
    // gtsam::Vector3 theta_PG(-1.1205, 0.076, -0.6636);  // @img 165.
    gtsam::Vector3 theta_PG(-1.1805, 0.106, -0.6636);  // @img 165.
    gtsam::Rot3 R_PG = gtsam::Rot3::Expmap(theta_PG);

    auto irNM = gtsam::noiseModel::Gaussian::Covariance(Sigma_omega);
    gtsam::InertiaRatiosManifoldFactor irfactor(Jsym, hij, GiBi, GjBj, WBi, WBj,
                                                irNM, R_PG, 0.5);
    irfactors_.push(irfactor);

    // Calculate an initial value for the angular momentum.
    gtsam::Pose3 T_WGi = T_WBi * T_GiBi.inverse();
    gtsam::Pose3 T_WGj = T_WBj * T_GjBj.inverse();
    Eigen::Vector3d omega_ij = ComputeAngularVelocity(
        T_WGi.rotation().matrix(), T_WGj.rotation().matrix(), 0.5);
    Eigen::Matrix3d Jmat = Eigen::Matrix3d::Identity();
    Jmat(0, 0) = 1.2207;  // From Setterfield17Phd,
    Jmat(1, 1) = 1.1672;  // Table 6.5, tri-axial.
    gtsam::Vector3 h_ij = gtsam::Vector3(Jmat * omega_ij);
    hbuffer_.push(h_ij);
    hkeys_.push(hij);

    // Record the state ID for the first ang mom variable.
    if (!initH_) {
      initH_ = true;
      initHiD_ = curStateId_;
      initHvalue_ = h_ij;  // To be used for setting up a single prior.
    }

    size_t inertiaRatiosStartID = 25;
    // size_t inertiaRatiosStartID = 60;
    // size_t inertiaRatiosStartID = 2500;
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues;
    if (curStateId_ == inertiaRatiosStartID) {
      // Initialize also the inertia ratios themselves. From Setterfield17Phd,
      // Table 6.5, ground truth tri-axial spin.

      // Initialize without any prior knowledge.
      // sph::InertiaRatios initJ(2.0, 1.0);
      // auto irNM = gtsam::noiseModel::Isotropic::Sigma(2, 20.00);

      // Initialize with prior information.
      sph::InertiaRatios initJ(1.2207, 1.1702);
      auto irNM = gtsam::noiseModel::Isotropic::Sigma(2, 0.01);

      newValues.insert(Jsym, initJ);
      gtsam::PriorFactor<sph::InertiaRatios> JPrior(Jsym, initJ, irNM);
      newFactors.push_back(JPrior);

      // Setup the priors for the angular momentum and bias chain.
      auto hpriorNM = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
      gtsam::PriorFactor<gtsam::Vector3> hPrior(gtsam::Symbol('h', initHiD_),
                                                initHvalue_, hpriorNM);
      newFactors.push_back(hPrior);
      newValues.insert(gtsam::Symbol('h', initHiD_), initHvalue_);

      auto bhpriorNM = gtsam::noiseModel::Isotropic::Sigma(3, 0.0001);
      gtsam::Vector3 zeroVec = (gtsam::Vector3() << 0.0, 0.0, 0.0).finished();
      gtsam::PriorFactor<gtsam::Vector3> bhPrior(
          gtsam::Symbol('R', initHiD_), gtsam::Vector3::Zero(), bhpriorNM);
      newFactors.push_back(bhPrior);
      newValues.insert(gtsam::Symbol('R', initHiD_), zeroVec);
    }

    if (curStateId_ >= inertiaRatiosStartID) {
      std::cout << "START ID! Sizes at ID: " << inertiaRatiosStartID
                << std::endl;
      std::cout << " irfactors_.size: " << irfactors_.size() << std::endl;
      std::cout << " bias buffersize: " << hbiasbuffer_.size() << std::endl;
      size_t hcounter = 0;
      while (!irfactors_.empty()) {
        std::cout << "Adding inertia ratios factor: " << std::endl;
        std::cout << "hcounter: " << hcounter << std::endl;
        std::cout << "irfactors: " << irfactors_.size() << std::endl;
        irfactors_.front().print();

        if (hcounter == 0 && irfactors_.size() > 1) {
          // Don't add new value, since it's been added already above. Also, we
          // cannot build a between factor because it's the first one.
          std::cout << "Skipping between factor..." << std::endl;

          // The inertia factor.
          newFactors.push_back(irfactors_.front());
          irfactors_.pop();
          hLatestKey_ = hkeys_.front();
        } else if ((hcounter == 0 && irfactors_.size() == 1) || hcounter > 0) {
          //   // Normal time step, gotta grab the previous solution for the
          //   bias and
          //   // create a between factor here.

          //   // The inertia factor.
          //   newFactors.push_back(irfactors_.front());
          //   irfactors_.pop();
          // } else {
          // Initial processing for the buffer after the first time step. Need
          // to add between factor and solve to get a solution for the bias.
          std::cout << "Emptying the buffer, adding a between factor\n";

          // Need to simultaneously add the inertia factor as well as the
          // between random walk factor before updating.

          // The inertia factor.
          newFactors.push_back(irfactors_.front());
          irfactors_.pop();

          gtsam::Symbol hCurrKey_j = hkeys_.front();
          // gtsam::Symbol hPrevKey_i('h', hCurrKey_j.index() - 1);
          gtsam::Symbol hPrevKey_i = hLatestKey_;
          gtsam::Symbol hbCurrKey_j('R', hCurrKey_j.index());
          gtsam::Symbol hbPrevKey_i('R', hPrevKey_i.index());

          // The random walk between factor.
          double hbtwnRW = 0.0001;
          auto hbtwnNM = gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(4) << 0.001, hbtwnRW, hbtwnRW, hbtwnRW)
                  .finished());
          gtsam::BetweenMomentumFactor hbtwnFactor(
              hPrevKey_i, hCurrKey_j, hbPrevKey_i, hbCurrKey_j, hbtwnNM);
          hbtwnFactor.print();
          newFactors.push_back(hbtwnFactor);

          // Insert new values for all new variables.
          newValues.insert(hkeys_.front(), hbuffer_.front());  // h_j
          newValues.insert(
              hbCurrKey_j,
              estimate_.at<gtsam::Vector3>(hbPrevKey_i));  // bias h_j

          hLatestKey_ = hkeys_.front();
        }

        // Compute estimate so that I can grab the previous bias.
        isam_->update(newFactors, newValues);
        estimate_ = isam_->calculateEstimate();
        newFactors.resize(0);
        newValues.clear();

        // Initialization values.
        // newValues.insert(hkeys_.front(), hbuffer_.front());
        // std::cout << "Adding initial values to symbol: " << hkeys_.front()
        //           << " = " << hbuffer_.front() << std::endl;
        // std::cout << "Norm: " << hbuffer_.front().norm() << std::endl;

        // Add prior to the angular momentum?
        // gtsam::PriorFactor<gtsam::Vector3> hPrior(hkeys_.front(),
        //                                           hbuffer_.front());
        // newFactors.push_back(hPrior);

        hcounter++;

        hkeys_.pop();
        hbuffer_.pop();
      }

      // Insert the new factors and the values.
      auto clock = sph::tick();
      isam_->update(newFactors, newValues);  // Inertia ratios.
      double time = sph::tock(clock);
      perf_ << time << " ";  // Update time 3/4.
    } else {
      // Just zero out the opt time for timesteps without inertia inclusion.
      perf_ << 0.0 << " ";  // Update time 3/4.
    }
  } else {
    // Just zero out the opt time for timesteps without inertia inclusion.
    perf_ << 0.0 << " ";  // Update time 3/4.
  }

  size_t principalAxesStartID = 25;
  // size_t principalAxesStartID = 60;
  if (curStateId_ == principalAxesStartID) {
    gtsam::NonlinearFactorGraph G;
    G.push_back(conicFactor_);
    gtsam::Values vals;
    gtsam::Vector3 theta_init(1.1798, -0.1076, 0.6612);
    // gtsam::Vector3 theta_init(1.1638, -0.1696, 0.7772);
    // gtsam::Vector3 theta_init(1.1295, -0.1461, 0.7913);
    // gtsam::Vector3 theta_init(0.0, 0.0, 0.0);

    gtsam::Rot3 R_init = gtsam::Rot3::Expmap(theta_init);
    auto cpfNM = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
    vals.insert(gtsam::Symbol('R', 0), R_init);
    G.addPrior(gtsam::Symbol('R', 0), R_init, cpfNM);

    auto clock = sph::tick();
    isam_->update(G, vals);
    double time = sph::tock(clock);
    perf_ << time << std::endl;
  } else if (curStateId_ > principalAxesStartID) {
    gtsam::Rot3 R_BG = estimate_.at<gtsam::Rot3>(gtsam::Symbol('R', 0));
    gtsam::Vector3 theta_BG = gtsam::Rot3::Logmap(R_BG);
    std::cout << "Current theta_BG: " << theta_BG.transpose() << std::endl;
    double avgTimestamp =
        0.5 * (frame.timestamp + raw_frames_[curStateId_ - 1].timestamp) /
        1000.0;  // in [s].
    // Try to get the cost for the current solution here.
    double cost = conicFactor_->error(estimate_);
    pao_ << avgTimestamp << " " << theta_BG.transpose() << " " << cost
         << std::endl;
    // Just zero out the opt time for timesteps after conic factor inclusion.
    perf_ << 0.0 << std::endl;  // Update time 4/4.
  } else {
    // Just zero out the opt time for timesteps before conic factor inclusion.
    perf_ << 0.0 << std::endl;  // Update time 4/4.
  }
  // TEMP end.

  // Write angular momentum estimates to file.
  if (estimate_.exists(gtsam::Symbol('h', curStateId_ - 1))) {
    if (hFirstPass_) {
      for (size_t i = 1; i < curStateId_ - 1; i++) {
        gtsam::Vector3 hval =
            estimate_.at<gtsam::Vector3>(gtsam::Symbol('h', i));
        momentum_ << raw_frames_[i].timestamp << " " << hval.transpose() << " "
                  << hval.norm() << std::endl;
      }
      hFirstPass_ = false;
    }

    gtsam::Vector3 hval =
        estimate_.at<gtsam::Vector3>(gtsam::Symbol('h', curStateId_ - 1));
    momentum_ << frame.timestamp << " " << hval.transpose() << " "
              << hval.norm() << std::endl;
  }

  // Add keyframe ID map from the current image frame ID, and update
  // buffers.
  imgIdToState_.insert(std::make_pair(frame.id, curStateId_));
  processed_frames_.push_back(processed_frame);
  raw_frames_.push_back(frame);
  times_.push_back(frame.timestamp);
  corresp_gm_.push_back(gm_.back());
  Pose3D T_GjBj = geometric_poses_.back() * T_BiBj;  // Gi == Gj == G.
  geometric_poses_.push_back(T_GjBj);
  curStateId_++;

  // TEMP start.
  std::cout << "+++++++++++++ Successful image update." << std::endl
            << std::endl;

  if (estimate_.exists(gtsam::Symbol('J', 0))) {
    sph::InertiaRatios Jopt =
        estimate_.at<sph::InertiaRatios>(gtsam::Symbol('J', 0));
    std::cout << "Optimal inertias: " << Jopt << std::endl;
    ratios_ << frame.timestamp << " " << Jopt.J1 << " " << Jopt.J2 << std::endl;
  }

  if (curStateId_ > 4) {
    ExportInertiaRatioParallel(frame.timestamp);
  }

  ExportFullG2oFile();
  std::string omegasOutFilename =
      "/Users/tonio/Documents/MATLAB/matlab/dynamics/omega-hist/out" +
      std::to_string(curStateId_ - 1) + ".omegasigma";
  ExportAngularVelocities(omegasOutFilename);
  // TEMP end.
}

/* ************************************************************************** */
void GraphManager::AddRelativePose(const RelativePoseMeasurement &rpm) {
  // If it's the first
}

/* ************************************************************************** */
void GraphManager::InitFactorGraph(const sph::StereoFrame &frame) {
  gtsam::NonlinearFactorGraph G;

  // Initial symbols.
  gtsam::Symbol pose_sym('T', curStateId_);
  gtsam::Symbol vel_sym('v', curStateId_);
  gtsam::Symbol bias_sym('b', curStateId_);
  gtsam::Symbol blob_sym('l', 0);  // Only one, and always zero index.

  // Noise models.
  // Set the noise models to be used for metrology priors (~3 deg orientation,
  // ~1 cm position [SPHERES Spec Sheet
  // http://ssl.mit.edu/spheres/library/SPHERES-Specifications.pdf], 2 mm/s
  // velocity [Uninformed guess Tim S.])
  auto ppNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.052, 0.052, 0.052, 0.01, 0.01, 0.01).finished());
  auto vpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.002, 0.002, 0.002).finished());
  // Gyro noise is taken from analysis of several test sessions by Jewison. The
  // accelerometer bias prior noise is set to a low value with little physical
  // meaning, since commanded forces are being used and thus should not have any
  // bias unless one axis has consistently weaker thrusters than another axis.
  // Bias order is accelBias, gyroBias, as in imuBias::ConstantBias.vector()
  auto bpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 0.004, 0.004, 0.004).finished());
  // Same as position from global metrology.
  auto lpNM = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);

  // Initialize using the latest global metrology measurement.
  gtsam::Pose3 T0(gtsam::Rot3(gm_.back().R), gtsam::Point3(gm_.back().t));
  gtsam::Velocity3 v0(gm_.back().v);
  gtsam::Point3 l0(Eigen::Vector3d::Zero());

  gtsam::PriorFactor<gtsam::Pose3> pPrior(pose_sym, T0, ppNM);
  gtsam::PriorFactor<gtsam::Velocity3> vPrior(vel_sym, v0, vpNM);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bPrior(bias_sym,
                                                          params_.bias0, bpNM);
  gtsam::PriorFactor<gtsam::Point3> lPrior(blob_sym, l0, lpNM);
  G.push_back(pPrior);
  G.push_back(vPrior);
  G.push_back(bPrior);
  G.push_back(lPrior);

  // Check to see if there are any blob track measurements to be added.
  if (frame.pv.vertigo_test_time != 0) {
    std::cout << " !!!!!!!!!!!!!!!!!!!! " << std::endl;
    std::cout << "we have blob to add during init" << std::endl;
    std::cout << " !!!!!!!!!!!!!!!!!!!! " << std::endl;
  }

  // Add initial values.
  gtsam::Values values;
  values.insert<gtsam::Pose3>(pose_sym, T0);
  values.insert<gtsam::Velocity3>(vel_sym, v0);
  values.insert(bias_sym, params_.bias0);
  values.insert<gtsam::Point3>(blob_sym, l0);

  // Add to iSAM2.
  isam_->update(G, values);

  // Record image ID in map.
  imgIdToState_.insert(std::make_pair(frame.id, curStateId_));

  // Advance the current state ID for the next keyframe.
  curStateId_++;
  corresp_gm_.push_back(gm_.back());
}

/* ************************************************************************** */
void GraphManager::InitGeomChain(const Pose3D &T_G0B0, const Pose3D &T_WB0) {
  gtsam::NonlinearFactorGraph G;

  // Initial symbols.
  gtsam::Symbol geom_sym('G', curStateId_ - 1);  // NOTE: Should be 0.
  gtsam::Symbol t_geom_sym('t', 0);              // There is only one node.

  /// Noise models.
  // NOTE(tonioteran) This should actually be equality, so we place a prior with
  // extremely low uncertainty. Could try to change to a PoseEquality factor.
  auto gpNM = gtsam::noiseModel::Isotropic::Sigma(6, 1e-10);  // [m,rad]
  // Noise model for prior on initial translation offset from geometric frame.
  auto tpNM = gtsam::noiseModel::Isotropic::Sigma(3, 3.0);  // [m]

  // Calculate the initial translation offset using initial pose estimate.
  gtsam::Point3 t_WwrtG0_G0 =
      gtsam::Point3(-(T_WB0.R.transpose() * T_WB0.t - T_G0B0.t));

  gtsam::PriorFactor<gtsam::Pose3> gPrior(geom_sym,
                                          gtsam::Pose3(T_G0B0.matrix()), gpNM);
  gtsam::PriorFactor<gtsam::Point3> tPrior(t_geom_sym, t_WwrtG0_G0, tpNM);

  G.push_back(gPrior);
  G.push_back(tPrior);

  // Add initial values.
  gtsam::Values values;
  values.insert<gtsam::Pose3>(geom_sym, gtsam::Pose3(T_G0B0.matrix()));
  values.insert<gtsam::Point3>(t_geom_sym, t_WwrtG0_G0);

  isam_->update(G, values);
  geometric_poses_.push_back(T_G0B0);
}

/* ************************************************************************** */
void GraphManager::AddFactors(const Pose3D &T_BiBj,
                              const sph::StereoFrame &frame) {
  // Get the involved symbols.
  // -- States.
  gtsam::Symbol pose_i('T', curStateId_ - 1), pose_j('T', curStateId_);
  gtsam::Symbol vel_i('v', curStateId_ - 1), vel_j('v', curStateId_);
  gtsam::Symbol bias_i('b', curStateId_ - 1), bias_j('b', curStateId_);
  // -- Geometric pose chain.
  gtsam::Symbol geom_i('G', curStateId_ - 1), geom_j('G', curStateId_);
  gtsam::Symbol t_geom('t', 0);    // Only one, same for all times.
  gtsam::Symbol blob_sym('l', 0);  // Only one, same for all times.

  gtsam::NonlinearFactorGraph G;
  gtsam::Values values;

  // Check to see if there are any blob track measurements to be added.
  if (frame.pv.vertigo_test_time != 0) {
    // std::cout << "Adding blob range and bearing factor." << std::endl;
    double range = frame.pv.t.norm() + params_.blob_range_bias;
    gtsam::Unit3 bearing(frame.pv.t);
    G.emplace_shared<
        gtsam::BearingRangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>>(
        pose_j, blob_sym, bearing, range, blobNoiseModel_,
        gtsam::Pose3(T_BC_.matrix()));
  }

  // Create IMU factor.
  gtsam::CombinedImuFactor imuf(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j,
                                *odometer_);
  // Compute initial values.
  auto Ti = isam_->calculateEstimate<gtsam::Pose3>(pose_i);
  auto vi = isam_->calculateEstimate<gtsam::Velocity3>(vel_i);
  auto bi = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(bias_i);
  gtsam::NavState xi(Ti, vi);
  gtsam::NavState xj = odometer_->predict(xi, bi);
  // Add to new factor graph.
  G.push_back(imuf);
  values.insert<gtsam::Pose3>(pose_j, xj.pose());
  values.insert<gtsam::Velocity3>(vel_j, xj.velocity());
  values.insert<gtsam::imuBias::ConstantBias>(bias_j, bi);

  // Create visual odometry factor.
  auto gNM = gtsam::noiseModel::Gaussian::Covariance(T_BiBj.cov);
  gtsam::BetweenFactor<gtsam::Pose3> vof(geom_i, geom_j,
                                         gtsam::Pose3(T_BiBj.matrix()), gNM);
  // Compute initial values.
  auto Gi = isam_->calculateEstimate<gtsam::Pose3>(geom_i);
  gtsam::Pose3 Gj = Gi * gtsam::Pose3(T_BiBj.matrix());
  // Add to new factor graph.
  G.push_back(vof);
  values.insert<gtsam::Pose3>(geom_j, Gj);

  // Create rotation kinematic factor. Noise model from TP_1511.
  auto rkfNM = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
  gtsam::RotationKinematicFactor rkf(pose_i, geom_i, pose_j, geom_j, t_geom,
                                     rkfNM);
  // Add to new factor graph.
  G.push_back(rkf);

  // Update the Bayes tree.
  auto clock = sph::tick();
  gtsam::ISAM2Result res = isam_->update(G, values);
  double uptime = sph::tock(clock);
  perf_ << uptime << " ";  // Update clock 1/4.
  for (size_t i = 0; i < params_.opt_iters; i++) isam_->update();
  estimate_ = isam_->calculateEstimate();

  WritePerfFile(frame.timestamp, &tree_, isam_.get(), isam_.get(), isam_.get(),
                uptime, uptime, uptime, res, res, res, estimate_, estimate_,
                estimate_);

  // Reset integration and set new bias.
  odometer_->resetIntegrationAndSetBias(bi);
}

/* ************************************************************************** */
void GraphManager::AttemptLoopClosure(
    const omsci::StereoFrame &processed_frame_j,
    const sph::StereoFrame &frame) {
  // Uniform distribution.
  std::uniform_int_distribution<size_t> dist(0, curStateId_ - 4);

  // Loop closures go from the current state 'j' to a previous state 'i', noting
  // that 'j' > 'i' must always hold.
  // NOTE: right now they are implemented as i->j. j->i was giving problems for
  // an unknown reason. Need to investigate more.

  bool success = false;
  // Attemp to match to a random past state a few times.
  std::cout << "  Loop closure attempt: " << std::flush;
  for (size_t k = 0; k < params_.max_lc_attempts; k++) {
    size_t i = dist(gen_);
    std::cout << "x" << i << ", " << std::flush;
    // Get the relative pose measurement and inliers by matching stereo pairs.
    sph::Pose3D T_CiCj;
    omsci::StereoFrame inlier_frame_i, inlier_frame_j;
    fe_->MatchPair(processed_frames_[i], processed_frame_j, &inlier_frame_i,
                   &inlier_frame_j, &T_CiCj);
    size_t n_inliers = inlier_frame_j.leftRightMatches3D.size();
    if (T_CiCj.R.determinant() != -1 && n_inliers > params_.min_lc_inliers) {
      // TODO(tonioteran) Add geometric sanity checks to avoid wrong loop
      // closure due to symmetry in SPHERES.
      std::cout << "... SUCCESS! ** Num inliers: "
                << inlier_frame_j.leftRightMatches3D.size() << std::endl
                << std::flush;

      std::cout << "  ++ Loop closure between x" << i << " and x" << curStateId_
                << " (prev img " << raw_frames_[i].id << ")" << std::endl
                << std::flush;

      Pose3D T_BiBj = T_BC_ * T_CiCj * T_BC_.inverse();  // Relative pose meas.

      gtsam::Symbol geom_i('G', i), geom_j('G', curStateId_);
      auto lcNM = gtsam::noiseModel::Gaussian::Covariance(T_BiBj.cov);
      gtsam::BetweenFactor<gtsam::Pose3> lcf(
          geom_i, geom_j, gtsam::Pose3(T_BiBj.matrix()), lcNM);

      gtsam::NonlinearFactorGraph G;
      G.push_back(lcf);
      auto clock = sph::tick();
      isam_->update(G);
      double time = sph::tock(clock);
      perf_ << time << " ";  // Update time 2/4.

      // Draw matched.
      fe_->OnlyDrawMatches(frame, processed_frame_j, inlier_frame_i,
                           inlier_frame_j, CV_RGB(0xFF, 0xFF, 0x00),
                           CV_RGB(0xFF, 0xFF, 0x00), &matched_);
      success = true;

      break;
    }
  }

  if (!success) perf_ << 0.0 << " ";
}

/* ************************************************************************** */
void GraphManager::GraphvizGraph(const std::string &out_file) const {
  // Get the factor graph.
  gtsam::NonlinearFactorGraph fg = isam_->getFactorsUnsafe();
  std::ofstream out(out_file);
  fg.saveGraph(out);
  out.close();
}

/* ************************************************************************** */
void GraphManager::GraphvizTree(const std::string &out_file) const {
  isam_->saveGraph(out_file);
}

/* ************************************************************************** */
void GraphManager::ExportFullG2oFile() const {
  // Write all relative pose measurements using current estimated values.
  std::ofstream outfull("full.g2o");

  size_t counter = 0;
  Pose3D P_WBi, P_WGi;
  gtsam::Symbol T('T', counter), G('G', counter);
  while (estimate_.exists(T)) {
    // Chaser's pose.
    gtsam::Pose3 T_WBj = estimate_.at<gtsam::Pose3>(T);
    Eigen::MatrixXd T_WBj_cov = isam_->marginalCovariance(T);
    Pose3D P_WBj(T_WBj.matrix(), T_WBj_cov);

    // Target's pose.
    gtsam::Pose3 T_WGj = estimate_.at<gtsam::Pose3>(G);
    Eigen::MatrixXd T_WGj_cov = isam_->marginalCovariance(G);
    Pose3D P_WGj(T_WGj.matrix(), T_WGj_cov);

    if (counter == 0) {
      // Add the connection between chains.
      Pose3D delta_B0G0 = P_WBj.inverse() * P_WGj;
      WriteG2oLine(&outfull, 2 * counter, 2 * counter + 1, delta_B0G0);
    } else {
      // Add edge to each individual chains.
      Pose3D delta_Bij = P_WBi.inverse() * P_WBj;
      Pose3D delta_Gij = P_WGi.inverse() * P_WGj;
      WriteG2oLine(&outfull, 2 * (counter - 1), 2 * counter, delta_Bij);
      WriteG2oLine(&outfull, 2 * (counter - 1) + 1, 2 * counter + 1, delta_Gij);

      // Add cross edges between two chains at each step.
      Pose3D delta_BjGj = P_WBj.inverse() * P_WGj;
      WriteG2oLine(&outfull, 2 * counter, 2 * counter + 1, delta_BjGj);
    }

    counter++;
    P_WBi = P_WBj;  // Save previous estimate to get delta pose.
    P_WGi = P_WGj;
    T = gtsam::Symbol('T', counter);
    G = gtsam::Symbol('G', counter);
  }
}

/* ************************************************************************** */
void GraphManager::ExportAngularVelocities(const std::string &filename) const {
  std::ofstream out(filename);
  size_t counter = 1;
  gtsam::Symbol WBi('T', counter - 1), WBj('T', counter);
  gtsam::Symbol GiBi('G', counter - 1), GjBj('G', counter);

  while (estimate_.exists(WBj)) {
    double deltaT = (times_[counter] - times_[counter - 1]) / 1000.0;  // [s]
    // Compute angular velocity estimate.
    if (deltaT < 0.8) {
      // Close enough pair of frames; extract geometric frame pose.
      // Compute the uncertainty as in Settefield PhD, Eq. A.13:
      gtsam::Pose3 T_GiBi = estimate_.at<gtsam::Pose3>(GiBi);
      gtsam::Matrix R_GiBi = T_GiBi.rotation().matrix();
      gtsam::Matrix R_GiBi_T = R_GiBi.transpose();
      gtsam::Matrix Sigma_R_GiBi =
          isam_->marginalCovariance(GiBi).block(0, 0, 3, 3);

      gtsam::Pose3 T_GjBj = estimate_.at<gtsam::Pose3>(GjBj);
      gtsam::Matrix Sigma_R_GjBj =
          isam_->marginalCovariance(GjBj).block(0, 0, 3, 3);

      gtsam::Pose3 T_WBi = estimate_.at<gtsam::Pose3>(WBi);
      gtsam::Matrix R_WBi = T_WBi.rotation().matrix();
      gtsam::Matrix Sigma_R_WBi =
          isam_->marginalCovariance(WBi).block(0, 0, 3, 3);

      gtsam::Pose3 T_WBj = estimate_.at<gtsam::Pose3>(WBj);
      gtsam::Matrix R_WBj = T_WBj.rotation().matrix();
      gtsam::Matrix Sigma_R_WBj =
          isam_->marginalCovariance(WBj).block(0, 0, 3, 3);

      // Uncertainty for the extracted angular velocity measurement.
      gtsam::Matrix Sigma_omega =
          (1.0 / 0.5) *
          (R_GiBi * (Sigma_R_GiBi + Sigma_R_WBi) * R_GiBi_T +
           R_GiBi * R_WBi.transpose() * R_WBj * (Sigma_R_WBj + Sigma_R_GjBj) *
               R_WBj.transpose() * R_WBi * R_GiBi_T);

      // Compute geometric frame poses and differentiate.
      gtsam::Pose3 T_WGi = T_WBi * T_GiBi.inverse();
      gtsam::Pose3 T_WGj = T_WBj * T_GjBj.inverse();
      Eigen::Vector3d omega_ij = ComputeAngularVelocity(
          T_WGi.rotation().matrix(), T_WGj.rotation().matrix(), 0.5);

      // Format:
      // time [s] | omega x,y,z | sigma^2 x | sigma^2 y | sigma^2 z
      out << (times_[counter] / 1000.0) << " " << omega_ij.transpose() << " "
          << Sigma_omega(0, 0) << " " << Sigma_omega(0, 1) << " "
          << Sigma_omega(0, 2) << " " << Sigma_omega(1, 1) << " "
          << Sigma_omega(1, 2) << " " << Sigma_omega(2, 2) << std::endl;
    }

    // Compute covariance.
    // Write to file.

    // Update counters.
    counter++;
    WBi = gtsam::Symbol('T', counter - 1);
    GiBi = gtsam::Symbol('G', counter - 1);
    WBj = gtsam::Symbol('T', counter);
    GjBj = gtsam::Symbol('G', counter);
  }

  out.close();
}

/* ************************************************************************** */
void GraphManager::ExportInertiaRatioParallel(const double time) const {
  size_t counter = 1;
  gtsam::Symbol WBi('T', counter - 1), WBj('T', counter);
  gtsam::Symbol GiBi('G', counter - 1), GjBj('G', counter);

  // Save the tgt body poses in inertial frame.
  std::vector<Eigen::Matrix3d> R_WBt;
  std::vector<Eigen::Vector3d> omegas;
  gtsam::Vector3 theta_PG(-1.1805, 0.106, -0.6636);  // @img 165.
  gtsam::Rot3 R_BG = gtsam::Rot3::Expmap(theta_PG);

  while (estimate_.exists(WBj)) {
    double deltaT = (times_[counter] - times_[counter - 1]) / 1000.0;  // [s]
    // Compute angular velocity estimate.
    if (deltaT < 0.8) {
      // Close enough pair of frames; extract geometric frame pose.
      // Compute the uncertainty as in Settefield PhD, Eq. A.13:
      gtsam::Pose3 T_GiBi = estimate_.at<gtsam::Pose3>(GiBi);
      gtsam::Matrix R_GiBi = T_GiBi.rotation().matrix();
      gtsam::Matrix R_GiBi_T = R_GiBi.transpose();
      gtsam::Matrix Sigma_R_GiBi =
          isam_->marginalCovariance(GiBi).block(0, 0, 3, 3);

      gtsam::Pose3 T_GjBj = estimate_.at<gtsam::Pose3>(GjBj);
      gtsam::Matrix Sigma_R_GjBj =
          isam_->marginalCovariance(GjBj).block(0, 0, 3, 3);

      gtsam::Pose3 T_WBi = estimate_.at<gtsam::Pose3>(WBi);
      gtsam::Matrix R_WBi = T_WBi.rotation().matrix();
      gtsam::Matrix Sigma_R_WBi =
          isam_->marginalCovariance(WBi).block(0, 0, 3, 3);

      gtsam::Pose3 T_WBj = estimate_.at<gtsam::Pose3>(WBj);
      gtsam::Matrix R_WBj = T_WBj.rotation().matrix();
      gtsam::Matrix Sigma_R_WBj =
          isam_->marginalCovariance(WBj).block(0, 0, 3, 3);

      // Uncertainty for the extracted angular velocity measurement.
      gtsam::Matrix Sigma_omega =
          (1.0 / 0.5) *
          (R_GiBi * (Sigma_R_GiBi + Sigma_R_WBi) * R_GiBi_T +
           R_GiBi * R_WBi.transpose() * R_WBj * (Sigma_R_WBj + Sigma_R_GjBj) *
               R_WBj.transpose() * R_WBi * R_GiBi_T);

      // Compute geometric frame poses and differentiate.
      gtsam::Pose3 T_WGi = T_WBi * T_GiBi.inverse();
      gtsam::Pose3 T_WGj = T_WBj * T_GjBj.inverse();
      Eigen::Vector3d omega_ij = ComputeAngularVelocity(
          T_WGi.rotation().matrix(), T_WGj.rotation().matrix(), 0.5);

      // Get the average rotation associated with the angular velocity.
      gtsam::Vector3 avgTh_WG = (0.5) * (gtsam::Rot3::Logmap(T_WGi.rotation()) +
                                         gtsam::Rot3::Logmap(T_WGj.rotation()));
      gtsam::Rot3 avgR_WG = gtsam::Rot3::Expmap(avgTh_WG);
      gtsam::Rot3 R_WBtgt = avgR_WG.compose(R_BG.inverse());
      R_WBt.push_back(R_WBtgt.matrix());
      omegas.push_back(R_BG.matrix() * omega_ij);

      // Format:
      // time [s] | omega x,y,z | sigma^2 x | sigma^2 y | sigma^2 z
      // out << (times_[counter] / 1000.0) << " " << omega_ij.transpose() << " "
      //     << Sigma_omega(0, 0) << " " << Sigma_omega(0, 1) << " "
      //     << Sigma_omega(0, 2) << " " << Sigma_omega(1, 1) << " "
      //     << Sigma_omega(1, 2) << " " << Sigma_omega(2, 2) << std::endl;
    }

    // Update counters.
    counter++;
    WBi = gtsam::Symbol('T', counter - 1);
    GiBi = gtsam::Symbol('G', counter - 1);
    WBj = gtsam::Symbol('T', counter);
    GjBj = gtsam::Symbol('G', counter);
  }

  // Build angular velocity matrix, compute inertia ratios parallel estimate.
  size_t N = omegas.size();
  Eigen::MatrixXd omegaMat = Eigen::MatrixXd::Zero(3, N);
  for (size_t i = 0; i < N; i++) {
    omegaMat.col(i) = omegas[i];
  }
  Eigen::Vector2d J = sph::ComputeInertiaRatiosParallel(omegaMat, R_WBt);

  jpar_ << time << " " << J(0) << " " << J(1) << std::endl;
}

}  // namespace sph
