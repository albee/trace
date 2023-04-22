/**
 * @file CommonTypes.h
 * @brief Custom SPHERES-related types.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Adapted by Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_COMMONTYPES_H_
#define SPH_VERTIGO_COMMONTYPES_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

// #include <opencv2/opencv.hpp>

namespace sph {

// Convenient shorthand for matrices and vectors:
template <typename T>
using Matrix3X = Eigen::Matrix<T, 3, Eigen::Dynamic>;
template <typename T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;
template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;
template <typename T>
using Matrix2X = Eigen::Matrix<T, 2, Eigen::Dynamic>;
template <typename T>
using Matrix2 = Eigen::Matrix<T, 2, 2>;
template <typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;
template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
template <typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

/// Trajectory consisiting of vector of Eigen-aligned 4x4 SE(3) matrices.
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
    Trajectory3;

/// Type of conic available for least squares fitting.
enum class ConicType { ELLIPSE, HYPERBOLA };

inline std::string ConicTypeString(const ConicType &t) {
  if (t == ConicType::ELLIPSE)
    return std::string("Ellipse");
  else
    return std::string("Hyperbola");
}

/// Direction of the conic.
enum class ConicDirection { XY, YX };

inline std::string ConicDirectionString(const ConicDirection &d) {
  if (d == ConicDirection::XY)
    return std::string("XY");
  else
    return std::string("YX");
}

/**
 * @brief Possible energy states for a rigid body.
 *
 * LE: low energy when h^2 > 2*Ek*J2, HE: high energy when h^2 < 2*Ek*J2.
 */
enum class EnergyState { LE, HE, ME };

inline std::string EnergyString(const EnergyState &e) {
  if (e == EnergyState::LE)
    return std::string("Low Energy");
  else if (e == EnergyState::HE)
    return std::string("High Energy");
  else
    return std::string("Medium Energy");
}

/**
 * @brief Possible inertia symmetries for a rigid body.
 *
 * TA: tri-axial. AS1: axis-symmetric J1 J2. AS3: axis-symmtric J2 J3.
 * FS: fully symmetric.
 */
enum class InertiaSymmetry { TA, AS1, AS3, FS };

inline std::string InertiaString(const InertiaSymmetry &i) {
  if (i == InertiaSymmetry::TA)
    return std::string("Tri-axial");
  else if (i == InertiaSymmetry::AS1)
    return std::string("Axis-symmetric1");
  else if (i == InertiaSymmetry::AS3)
    return std::string("Axis-symmetric3");
  else
    return std::string("Fully-symmetric");
}

/**
 * @brief Scaled and unbiased SPHERES IMU data message [rad/s, m/s^2].
 */
struct MsgImuData {
  double vertigo_test_time;  ///< [ms]
  double sph_test_time;      ///< [ms]
  Eigen::Vector3d gyro;      ///< [rad/s]
  Eigen::Vector3d accel;     ///< [m/s^2/s]

  /// Handy print function for nice std out formatting.
  inline friend std::ostream &operator<<(std::ostream &os,
                                         const MsgImuData &m) {
    os << "IMU data message." << std::endl
       << "  - VERTIGO test time: " << m.vertigo_test_time << std::endl
       << "  - SPHERES test time: " << m.sph_test_time << std::endl
       << "  - gyro^T  : " << m.gyro.transpose() << std::endl
       << "  - accel^T : " << m.accel.transpose() << std::endl;
    return os;
  }
};

/**
 * @brief SPHERES on/off thruster times [ms].
 */
struct MsgThrusterTimes {
  // NOTE(tonioteran) I don't think this is necessary. Test and remove.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  unsigned int vertigo_test_time;      ///< [ms].
  unsigned int sph_test_time;          ///< [ms].
  Eigen::Matrix<int, 12, 1> on_time;   ///< Thruster open time stamp [ms].
  Eigen::Matrix<int, 12, 1> off_time;  ///< Thruster close time stamp [ms].

  /// Handy print function for nice std out formatting.
  inline friend std::ostream &operator<<(std::ostream &os,
                                         const MsgThrusterTimes &m) {
    os << "Thruster firing times message." << std::endl
       << "  - VERTIGO test time: " << m.vertigo_test_time << std::endl
       << "  - SPHERES test time: " << m.sph_test_time << std::endl
       << "  - on times^T  : " << m.on_time.transpose() << std::endl
       << "  - off times^T : " << m.off_time.transpose() << std::endl;
    return os;
  }
};

/**
 * @brief SPHERES global metrology message.
 */
struct MsgGlobalMet {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double vertigo_test_time;    ///< [ms].
  unsigned int sph_test_time;  ///< [ms].
  Eigen::Vector3d t;           ///< Position.
  Eigen::Vector3d v;           ///< Velocity.
  Eigen::Quaterniond q;        ///< Attitude.
  Eigen::Matrix3d R;           ///< Attitude in rotation matrix form.
  Eigen::Vector3d omega;       ///< Angular rate.

  /// Handy print function for nice std out formatting.
  inline friend std::ostream &operator<<(std::ostream &os,
                                         const MsgGlobalMet &m) {
    os << "Global metrology message." << std::endl
       << "  - VERTIGO test time: " << m.vertigo_test_time << std::endl
       << "  - SPHERES test time: " << m.sph_test_time << std::endl
       << "  - position^T: " << m.t.transpose() << std::endl
       << "  - velocity^T: " << m.v.transpose() << std::endl
       << "  - attitude quat [w,x,y,z]: [" << m.q.w() << "," << m.q.x() << ","
       << m.q.y() << "," << m.q.z() << "]" << std::endl
       << "  - attitude R:\n"
       << m.R << std::endl
       << "  - omega^T: " << m.omega.transpose() << std::endl;
    return os;
  }
};

/**
 * @brief SPHERES blob message with position and velocity.
 */
struct MsgPosVel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double vertigo_test_time;  ///< [ms].
  Eigen::Vector3d t;         ///< Position.
  Eigen::Vector3d v;         ///< Velocity.

  /// Handy print function for nice std out formatting.
  inline friend std::ostream &operator<<(std::ostream &os, const MsgPosVel &m) {
    os << "PosVel message." << std::endl
       << "  - VERTIGO test time: " << m.vertigo_test_time << std::endl
       << "  - position^T: " << m.t.transpose() << std::endl
       << "  - velocity^T: " << m.v.transpose() << std::endl;
    return os;
  }
};

// /**
//  * @brief Structure to encapsulate feature information for an image pair.
//  */
//  /*
// struct StereoInfo {
//   std::vector<cv::KeyPoint> keypts_left, keypts_right;  ///< Keypoints.
//   cv::Mat descr_left, descr_right;                      ///< Descriptors.
//   std::vector<cv::DMatch> matches;                      ///< Correspondences.
//   std::vector<Eigen::Vector3d> matches3D;               ///< 3D points.
//   std::vector<Eigen::Matrix3d> cov_matches3D;           ///< Uncertainties.
//   Eigen::Vector4d mask_params;                          ///< Target rect mask.
//   int id;                                               ///< Identifier.
// };
//
// /**
//  * @brief Structure to encapsulate stereo images and related information.
//  */
//  /*
// struct StereoFrame {
//   cv::Mat img_left, img_right, img_matches;  ///< Image containers.
//   StereoInfo info;                           ///< Features infoormation.
//   bool processed = false;                    ///< Features detected&extracted.
//   bool matched = false;                      ///< Left/right matched.
//   double timestamp;                          ///< VERTIGO time stamp.
//   int id;                                    ///< Stereo frame number.
//   int match_id;                              ///< Id of matched frame.
//   MsgPosVel pv;                              ///< Corresponding posVel.
//   friend std::ostream &operator<<(std::ostream &os, const StereoFrame &sf) {
//     os << "StereoFrame object #" << sf.id << std::endl
//        << " processed : " << sf.processed << std::endl
//        << " timestamp : " << sf.timestamp << std::endl
//        << " left,right: " << sf.img_left.size() << "," << sf.img_right.size()
//        << " posvel: \n"
//        << sf.pv << std::endl;
//     if (sf.processed) {
//       os << " info: " << std::endl
//          << "  keypts,descrp left : " << sf.info.keypts_left.size() << ","
//          << sf.info.descr_left.size() << std::endl
//          << "  descrp left: rows=" << sf.info.descr_left.rows
//          << ", cols=" << sf.info.descr_left.cols << std::endl
//          << "  keypts,descrp right: " << sf.info.keypts_right.size() << ","
//          << sf.info.descr_right.size() << std::endl
//          << "  matches  : " << sf.info.matches.size() << std::endl
//          << "  matches3d: " << sf.info.matches3D.size() << std::endl
//          << "  cov matches3d: " << sf.info.cov_matches3D.size() << std::endl;
//     }
//     return os;
//   }
// };
// */
//
// /**
//  * @param Structure to hold all depth information for an image.
//  */
//  /*
// struct DepthFrame {
//   cv::Mat depth_map;         ///< Float map holding depth values.
//   cv::Mat depth_img;         ///< Corresponding 8UC1 image of depth map.
//   cv::Mat disp_map;          ///< 16S map with 4 fractional bits.
//   cv::Mat disp_img;          ///< Corresponding 8UC1 image of disparity map.
//   Eigen::Vector3d centroid;  ///< Location of depth map's centroid.
// };
// */
//
// /**
//  * @brief Stereo camera type to hold calibration information.
//  */
//  /*
// struct StereoCamera {
//   double f, cx, cy;      ///< Focal length, principal point coords (left) [px].
//   double tx;             ///< Stereo baseline [m].
//   int width, height;     ///< Image size [px].
//   Eigen::Matrix3d covS;  ///< CovMat for CamCoords s = [uL uR vL]' [px^2].
//   friend std::ostream &operator<<(std::ostream &os, const StereoCamera &sc) {
//     os << " * Stereo Camera struct with calib params: " << std::endl
//        << "  - f      : " << sc.f << std::endl
//        << "  - Tx     : " << sc.tx << std::endl
//        << "  - (cx,cy): (" << sc.cx << "," << sc.cy << ")" << std::endl
//        << "  - (wd,ht): (" << sc.width << "," << sc.height << ")" << std::endl;
//     return (os);
//   }
// };
// */

struct TransformationSE3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Rotation matrix
  Eigen::Matrix3d R;

  // Translation [m]
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> t;

  // Covariance matrix (expected value of [delTheta delT] [delTheta delT]')
  Eigen::Matrix<double, 6, 6, Eigen::DontAlign> cov;

  // Constructor from rotation and translation
  TransformationSE3(
      Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity(),
      Eigen::Vector3d translation = Eigen::Vector3d::Zero(),
      Eigen::Matrix<double, 6, 6> covariance =
          Eigen::Matrix<double, 6, 6>::Zero())
      : R(rotationMatrix), t(translation), cov(covariance) {}

  // Constructor from transformation matrix
  TransformationSE3(Eigen::Matrix4d transformationMatrix,
                    Eigen::Matrix<double, 6, 6> covariance =
                        Eigen::Matrix<double, 6, 6>::Zero())
      : R(transformationMatrix.block(0, 0, 3, 3)),
        t(transformationMatrix.block(0, 3, 3, 1)),
        cov(covariance) {}

  // Create a skew symmetric cross-product matrix
  Eigen::Matrix3d skew(const Eigen::Vector3d vec) {
    Eigen::Matrix3d skewMatrix;
    skewMatrix << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return skewMatrix;
  }

  // Return the inverse of the SE3 transform. See logbook #3 pp 147
  TransformationSE3 inverse() {
    TransformationSE3 inverse;

    inverse.R = R.transpose();
    inverse.t = -R.transpose() * t;

    // updated - test this
    // Determine covariance (see logbook #3 pp 147)
    inverse.cov = Eigen::Matrix<double, 6, 6>::Zero();
    inverse.cov.block(0, 0, 3, 3) = R * cov.block(0, 0, 3, 3) * R.transpose();
    inverse.cov.block(3, 3, 3, 3) =
        R * cov.block(3, 3, 3, 3) * R.transpose() -
        skew(t) * R * cov.block(0, 0, 3, 3) * R.transpose() * skew(t) -
        R * cov.block(3, 0, 3, 3) * R.transpose() * skew(t) +
        skew(t) * R * cov.block(0, 3, 3, 3) * R.transpose();
    inverse.cov.block(0, 3, 3, 3) =
        R * cov.block(0, 3, 3, 3) * R.transpose() -
        R * cov.block(0, 0, 3, 3) * R.transpose() * skew(t);
    inverse.cov.block(3, 0, 3, 3) = inverse.cov.block(0, 3, 3, 3).transpose();

    /* old - known to work
    inverse.cov = Eigen::Matrix<double,6,6>::Zero();
    inverse.cov.block(0,0,3,3) = cov.block(0,0,3,3);
    inverse.cov.block(3,3,3,3) = R.transpose() * cov.block(3,3,3,3) * R;
    */

    return inverse;
  }

  // Get the 4x4 SE(3) matrix
  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();

    matrix.block(0, 0, 3, 3) = R;
    matrix.block(0, 3, 3, 1) = t;
    matrix(3, 3) = 1.0;

    return matrix;
  }

  // Get the quaternion
  Eigen::Quaterniond quat() const {
    // Get quaternion from rotation matrix (same way as in omsci::rot2quat)
    Eigen::Quaterniond q(R.transpose());
    return q;
  }

  // Get the rotation angle magnitude in radians
  double getRotAngle() const {
    // Get quaternion from rotation matrix (same way as in omsci::rot2quat)
    Eigen::Quaterniond q(R.transpose());
    return 2 * std::acos(q.w());
  }

  // Compose transformations; follows pose matrix order PAtoC =
  // PAtoB.compose(PBtoC)
  TransformationSE3 compose(const TransformationSE3 &P2) {
    TransformationSE3 comp;

    comp.R = R * P2.R;
    comp.t = t + R * P2.t;

    // updated - in testing
    // Determine covariance (see logbook #3 pp 148-149)
    /*
      comp.cov = Eigen::Matrix<double, 6, 6>::Zero();
      comp.cov.block(0, 0, 3, 3) =
      P2.R.transpose() * cov.block(0, 0, 3, 3) * P2.R +
      P2.cov.block(0, 0, 3, 3);
      comp.cov.block(3, 3, 3, 3) =
      P2.R.transpose() * cov.block(3, 3, 3, 3) * P2.R +
      P2.cov.block(3, 3, 3, 3) -
      P2.R.transpose() * skew(P2.t) * cov.block(0, 0, 3, 3) * skew(P2.t) *
      P2.R +
      P2.R.transpose() * cov.block(3, 0, 3, 3) * skew(P2.t) * P2.R -
      P2.R.transpose() * skew(P2.t) * cov.block(0, 3, 3, 3) * P2.R;
      comp.cov.block(0, 3, 3, 3) =
      cov.block(0, 3, 3, 3) +
      P2.R.transpose() * cov.block(0, 3, 3, 3) * P2.R +
      P2.R.transpose() * cov.block(0, 0, 3, 3) * skew(P2.t) * P2.R;
      comp.cov.block(3, 0, 3, 3) = comp.cov.block(0, 3, 3, 3).transpose();
     */

    // TEMP switch back.
    ///* old -known to work
    comp.cov = Eigen::Matrix<double, 6, 6>::Zero();
    comp.cov.block(0, 0, 3, 3) =
        R * P2.cov.block(0, 0, 3, 3) * R.transpose() + cov.block(0, 0, 3, 3);
    comp.cov.block(3, 3, 3, 3) =
        R * P2.cov.block(3, 3, 3, 3) * R.transpose() + cov.block(3, 3, 3, 3);
    //*/
    // TEMP

    return comp;
  }

  // Alternate syntax for composition
  TransformationSE3 operator*(const TransformationSE3 &P2) {
    return compose(P2);
  }

  // Function for converting from a 2D array of floats to a rotation matrix
  static Eigen::Matrix<double, 3, 3> rotArray2rotMatrix(
      const float rotArr[][3]) {
    Eigen::Matrix<double, 3, 3> R;
    R << rotArr[0][0], rotArr[0][1], rotArr[0][2], rotArr[1][0], rotArr[1][1],
        rotArr[1][2], rotArr[2][0], rotArr[2][1], rotArr[2][2];
    return R;
  }

  // For printing to std::cout and other streams
  friend std::ostream &operator<<(std::ostream &stream,
                                  TransformationSE3 &trans) {
    // Get quaternion from rotation matrix (same way as in omsci::rot2quat)
    Eigen::Quaterniond q(trans.R.transpose());

    return stream << trans.t(0) << "," << trans.t(1) << "," << trans.t(2)
                  << ",    " << q.x() << "," << q.y() << "," << q.z() << ","
                  << q.w() << " (" << trans.t.norm() << " m, "
                  << ((2 * std::acos(q.w())) * 180 / 3.14159) << " deg)";
  }
};

typedef TransformationSE3 Pose3D;  ///< Handy typedef.

/**
 * @brief Useful for building VO relative pose measurement and loop closures.
 */
struct RelativePoseMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Poses involved.
  int i, j;

  // Timestamps for poses [ms].
  double time_i, time_j;

  // Inliers used to compute.
  int inliers;

  // Actual relative pose transformation.
  Pose3D relative_pose;
};

}  // namespace sph
//
// namespace omsci {
//
// struct StereoFrame {  // stereo frame type
//   std::vector<cv::KeyPoint> leftKeypoints,
//       rightKeypoints;  // all detected keypoints
//   std::vector<int> leftRowLookupTable,
//       rightRowLookupTable;  // lookup table for index of keypoints starting at a
//                             // row
//   cv::Mat leftDescriptors,
//       rightDescriptors;  // all detected keypoint descriptors
//   std::vector<cv::DMatch>
//       leftRightMatches;  // all keypoints matched from left to right
//   std::vector<Eigen::Vector3d>
//       leftRightMatches3D;  // 3D coords all keypoints matched from left to right
//                            // in left camera frame [m]
//   std::vector<Eigen::Matrix3d>
//       covLeftRightMatches3D;  // the covariance of all the left to right matches
//                               // [m^2]
//   bool used = false;          // whether or not the stereo frame has been "used"
//
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// Comparison structure / function used for sorting keypoints given indicies
// struct compareKeypointIndexY {
//   std::vector<cv::KeyPoint> keypoints;
//   compareKeypointIndexY(std::vector<cv::KeyPoint> keyPts) {
//     this->keypoints = keyPts;
//   }
//   bool operator()(size_t index1, size_t index2) {
//     return keypoints[index1].pt.y < keypoints[index2].pt.y;
//   }
// };
//
// // Type of feature detectors / extractors / matchers
// enum FeatureDetectorType {
//   FDT_SIFT,
//   FDT_SURF,
//   FDT_ORB,
//   FDT_BRISK,
//   FDT_FAST,
//   FDT_GFTT,
//   FDT_STAR,
//   FDT_MSER,
//   FDT_HARRIS,
//   FDT_Dense,
//   FDT_SimpleBlob,
//   FDT_AKAZE
// };
// enum DescriptorExtractorType {
//   DET_SIFT,
//   DET_SURF,
//   DET_ORB,
//   DET_BRISK,
//   DET_BRIEF,
//   DET_FREAK,
//   DET_AKAZE
// };
// enum DescriptorMatcherType {
//   DMT_BruteForce,
//   DMT_BruteForce_L1,
//   DMT_BruteForce_Hamming,
//   DMT_BruteForce_Hamming_2,
//   DMT_FlannBased,
//   DMT_AKAZE
// };
//
// // Create a skew symmetric cross-product matrix
// Eigen::Matrix3d skew(const Eigen::Vector3d vec);
//
// // Convert from quaternion to rotation matrix, using SPHERES conventions
// Eigen::Matrix3d quat2rot(const Eigen::Quaterniond quat);
//
// // Convert from quaternion to rotation matrix, using SPHERES conventions
// Eigen::Matrix3d quat2rot(const Eigen::Vector4d quat);
//
// // Convert from rotation matrix to quaternion, using SPHERES conventions
// Eigen::Quaterniond rot2quat(const Eigen::Matrix3d rot);
//
// }  // namespace omsci

#endif  // SPH_VERTIGO_COMMONTYPES_H_
