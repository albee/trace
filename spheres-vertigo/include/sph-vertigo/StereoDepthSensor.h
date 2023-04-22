/**
 * @file StereoDepthSensor.h
 * @brief Process stereo frames to obtain disparity and depth maps.
 * @date November 22, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 *
 * Based on Tim P Setterfield's VERTIGO code base.
 */

#ifndef SPH_VERTIGO_STEREODEPTHSENSOR_H_
#define SPH_VERTIGO_STEREODEPTHSENSOR_H_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @brief Disparity-based depth sensing class.
 *
 * Stereo-based class for computing depth and disparity maps using intrinsic and
 * extrinsix calibration parameters from a stereo camera and block matching
 * algorithms.
 */
class StereoDepthSensor {
 private:
  /// Holds camera's calibration parameters and error characteristics
  std::shared_ptr<StereoCamera> stereo_cam_ = std::make_shared<StereoCamera>();

  /// Default block matching tunable parameters
  const int num_disp_{16 * 5}, block_size_{21};

  /// OpenCV's auxiliary stereo block matcher
  cv::Ptr<cv::StereoBM> stereoBM_{cv::StereoBM::create(num_disp_, block_size_)};

  /// min,max depth [m] and disparity [px]
  double min_depth_, max_depth_, min_disp_, max_disp_;

 public:
  /* Constructors */
  //! Constructor using default block matching parameters
  explicit StereoDepthSensor(std::shared_ptr<StereoCamera> stereo_camera);

  //! Overloaded constructor with custom block size and number of disparities
  StereoDepthSensor(std::shared_ptr<StereoCamera> stereo_camera,
                    const int num_disp, const int block_size);

  /* Destructors */
  virtual ~StereoDepthSensor();

  /**
   * @brief Compute disparity via OpenCV 'StereoBM' from left and right images.
   * @param[in] left Stero image.
   * @param[in] right Stereo image.
   * @param[out] disp Disparity image.
   *
   * NOTE(tonioteran) disp needs to be CV_16S (signed 16 bits), fixed-point
   * arithmetic output.
   */
  void getDisparity(const cv::Mat &left, const cv::Mat &right,
                    cv::Mat *disp) const;

  /**
   * @brief Computes depth map from disparity map via triangulation parameters.
   * @param[in] disp Per pixel disparity map (CV_16S).
   * @param[out] depth_map Per pixel depth map (CV_32F).
   *
   * NOTE(tonioteran): inside we divide by 16: 16-bit fixed-point with 4
   * fractional bits (2^4=16).
   */
  void getDepthMap(const cv::Mat &disp, cv::Mat *depth_map) const;

  //! Computes depth map from 32F disparity map using triangulation parameters
  void getDepthMap32F(const cv::Mat &disp32, cv::Mat &depth_map) const;

  //! Converts float depth map into imshow-able 8bit-1channel image
  void getDepthMapImage(const cv::Mat &depth_map, cv::Mat &depth_im) const;

  //! Calculates 8UC1 cv::Mat disparity image from left and right source images
  void getDisparityImage(const cv::Mat &left, const cv::Mat &right,
                         cv::Mat &disp) const;

  //! Converts 16S disparity map to 8UC1 cv::Mat image
  void getDisparityImage(const cv::Mat &disparity, cv::Mat &disp_im) const;

  //! Calculates centroid from a float depth map
  Eigen::Vector3d getCentroid(const cv::Mat &depth_map) const;

  //! Projects calculated depth centroid from depth map onto image plane
  Eigen::Vector3d getCentroidImage(const cv::Mat &depth_im,
                                   cv::Mat &centroid_im) const;

  //! Projects input centroid onto output image plane
  void getCentroidImage(const cv::Mat &depth_im,
                        const Eigen::Vector3d &centroid,
                        cv::Mat &centroid_im) const;

  //! Overloaded function that just projects input centroid onto input img
  void getCentroidImage(const Eigen::Vector3d &centroid,
                        cv::Mat &centroid_im) const;

  //! Mask input image using min and max threshold values
  void maskImageDepthThresh(const double min_depth, const double max_depth,
                            const StereoFrame &frame,
                            StereoFrame &out_frame) const;

  //! Masks disparity map using max depth value
  void maskByDepth(const double max_depth, const StereoFrame &frame,
                   DepthFrame &depth_frame) const;

  //! Computes disparity and depth maps and images, as well as centroid
  void fillDepthFrame(const StereoFrame &frame, DepthFrame &depth_frame) const;
  //! Overloaded function to allow for depth masking using max input value
  void fillDepthFrame(const StereoFrame &frame, DepthFrame &depth_frame,
                      const double max_depth) const;

  //! Calculates rectangular mask of target in image frame
  /*!
    Using the specified target dimensions in meters (`target_dim`) and the
    location of the centroid (all in the left camera frame), calculates the
    projection in both left, and right image planes of a `target_dim`^2 size
    square centered in centroid. Returns mask of frame's img size, with 255
    values inside the region of interest (i.e., the projected square).
    \return 4x1 vector with (sqr_x left, sqr_x right, sqr_y, sqr_length)
   */
  Eigen::Vector4d getTargetMasks(const StereoFrame &frame,
                                 const Eigen::Vector3d &centroid,
                                 const double target_dim, cv::Mat &mask_left,
                                 cv::Mat &mask_right) const;

}; /* StereoDepthSensor */

}  // namespace sph

#endif  // SPH_VERTIGO_STEREODEPTHSENSOR_H_
