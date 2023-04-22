/**
 *  @file  RotationKinematicFactor.cpp
 *  @author Tim Setterfield
 **/

#include "mit_slam/RotationKinematicFactor.h"

#include <iostream>

namespace mit_slam {

using namespace std;

/* ************************************************************************* */
RotationKinematicFactor::RotationKinematicFactor(gtsam::Key PWtoBi, gtsam::Key PGitoBi,
                                                 gtsam::Key PWtoBj, gtsam::Key PGjtoBj,
                                                 gtsam::Key tGtoBt_G,
                                                 const gtsam::SharedNoiseModel& model)
    : Base(model, PWtoBi, PGitoBi, PWtoBj, PGjtoBj, tGtoBt_G) {}

/* ************************************************************************* */
gtsam::Vector RotationKinematicFactor::evaluateError(
    const gtsam::Pose3& PWtoBi, const gtsam::Pose3& PGitoBi, const gtsam::Pose3& PWtoBj,
    const gtsam::Pose3& PGjtoBj, const gtsam::Point3& tGtoBt_G, boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2, boost::optional<gtsam::Matrix&> H3,
    boost::optional<gtsam::Matrix&> H4, boost::optional<gtsam::Matrix&> H5) const {
  /** Rotations */
  gtsam::Rot3 RBitoW = PWtoBi.rotation();
  gtsam::Rot3 RGitoBi = PGitoBi.rotation().inverse();
  gtsam::Rot3 RBjtoW = PWtoBj.rotation();
  gtsam::Rot3 RGjtoBj = PGjtoBj.rotation().inverse();
  /** Translations */
  gtsam::Point3 tWtoBi_W = PWtoBi.translation();
  gtsam::Point3 tGitoBi_Gi = PGitoBi.translation();
  gtsam::Point3 tWtoBj_W = PWtoBj.translation();
  gtsam::Point3 tGjtoBj_Gj = PGjtoBj.translation();

  /** Jacobians */
  gtsam::Point3 xi = RBitoW * RGitoBi * (tGitoBi_Gi - tGtoBt_G);
  gtsam::Point3 xj = RBjtoW * RGjtoBj * (tGtoBt_G - tGjtoBj_Gj);
  if (H1) {  // dKappa/dRBitoW, and dKappa/tWtoBi_W
    *H1 = (gtsam::Matrix(3, 6) << -gtsam::skewSymmetric(xi.x(), xi.y(), xi.z()) *
                               RBitoW.matrix(),
           -RBitoW.matrix())
              .finished();
  }
  if (H2) {  // dKappa/dRBitoGi, and dKappa/tGitoBi_Gi
    *H2 = (gtsam::Matrix(3, 6) << gtsam::skewSymmetric(xi.x(), xi.y(), xi.z()) *
                               RBitoW.matrix(),
           RBitoW.matrix())
              .finished();
  }
  if (H3) {  // dKappa/dRBjtoW, and dKappa/tWtoBj_W
    *H3 = (gtsam::Matrix(3, 6) << -gtsam::skewSymmetric(xj.x(), xj.y(), xj.z()) *
                               RBjtoW.matrix(),
           RBjtoW.matrix())
              .finished();
  }
  if (H4) {  // dKappa/dRBjtoGj, and dKappa/tGjtoBj_Gj
    *H4 = (gtsam::Matrix(3, 6) << gtsam::skewSymmetric(xj.x(), xj.y(), xj.z()) *
                               RBjtoW.matrix(),
           -RBjtoW.matrix())
              .finished();
  }
  if (H5) {  // dKappa/tGtoBt_G
    *H5 = (gtsam::Matrix(3, 3) << RBjtoW.matrix() * RGjtoBj.matrix() -
                               RBitoW.matrix() * RGitoBi.matrix())
              .finished();
  }

  /**
   * The vector addition below will completes a loop and will equal the zero
   * vector when the kinematic constraints are satisfied.
   */
  return RBitoW * RGitoBi * (tGitoBi_Gi - tGtoBt_G) +
         RBjtoW * RGjtoBj * (tGtoBt_G - tGjtoBj_Gj) + (tWtoBj_W - tWtoBi_W);
}

/* ************************************************************************* */
void RotationKinematicFactor::print(const std::string& s,
                                    const gtsam::KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool RotationKinematicFactor::equals(const RotationKinematicFactor& factor,
                                     double tol) const {
  return Base::equals(factor);
}

}  // end namespace gtsam
