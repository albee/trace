/**
 * @file StereoDepthSensor.cpp
 * @brief process stereo frames to obtain disparity and depth maps
 * @date November 22, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 *
 * Based on Tim P Setterfield's VERTIGO code base.
 */

#include "sph-vertigo/StereoDepthSensor.h"

namespace sph {

/* ************************************************************************** */
StereoDepthSensor::StereoDepthSensor(
    std::shared_ptr<StereoCamera> stereo_camera)
    : stereo_cam_(std::move(stereo_camera)) {
  std::cout << "+ Created StereoDepthSensor object" << std::endl;
}

/* ************************************************************************** */
StereoDepthSensor::StereoDepthSensor(
    std::shared_ptr<StereoCamera> stereo_camera, const int num_disp,
    const int block_size)
    : stereo_cam_(std::move(stereo_camera)),
      num_disp_(num_disp),
      block_size_(block_size),
      stereoBM_(cv::StereoBM::create(num_disp, block_size)) {
  std::cout << "+ Created StereoDepthSensor object with custom params"
            << std::endl;
}

/* ************************************************************************** */
StereoDepthSensor::~StereoDepthSensor() {}

/* ************************************************************************** */
void StereoDepthSensor::getDisparity(const cv::Mat &left, const cv::Mat &right,
                                     cv::Mat *disp) const {
  // NOTE disp needs to be CV_16S (signed 16 bits), fixed-point arthm output
  *disp = cv::Mat(left.rows, left.cols, CV_16S);
  stereoBM_->compute(left, right, *disp);
}

/* ************************************************************************** */
void StereoDepthSensor::getDepthMap(const cv::Mat &disp,
                                    cv::Mat *depth_map) const {
  *depth_map = cv::Mat::zeros(disp.rows, disp.cols, CV_32F);
  // multiply only once for convenience
  float Txf =
      stereo_cam_->tx * stereo_cam_->f;  // [m]*[px] to then be / by [px]

  // Reproject all pixels
  for (int r = 0; r < disp.rows; r++) {
    for (int c = 0; c < disp.cols; c++) {
      if (disp.at<short>(r, c) > 0) {  // neg values are invalid
        // divide by 16: 16-bit fixed-point with 4 fractional bits (2^4=16)
        auto disparity_val = static_cast<float>(disp.at<short>(r, c)) / 16.0;
        float depth_val = Txf / disparity_val;  // [m*px]/[px] = [m] (meters)
        depth_map->at<float>(r, c) = depth_val;
      }
    }
  }
}

/* ************************************************************************** */
void StereoDepthSensor::getDepthMap32F(const cv::Mat &disp32,
                                       cv::Mat &depth_map) const {
  depth_map = cv::Mat::zeros(disp32.rows, disp32.cols, CV_32F);
  // multiply only once for convenience
  float Txf =
      stereo_cam_->tx * stereo_cam_->f;  // [m]*[px] to then be / by [px]

  // Reproject all pixels
  for (int r = 0; r < disp32.rows; r++) {
    for (int c = 0; c < disp32.cols; c++) {
      if (disp32.at<short>(r, c) > 0) {  // neg values are invalid
        auto disparity_val = static_cast<float>(disp32.at<short>(r, c));
        float depth_val = Txf / disparity_val;  // [m*px]/[px] = [m] (meters)
        depth_map.at<float>(r, c) = depth_val;
      }
    }
  }
}

/* ************************************************************************** */
void StereoDepthSensor::getDepthMapImage(const cv::Mat &depth_map,
                                         cv::Mat &depth_im) const {
  cv::Mat depth8U = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC1);
  double min_val, max_val;
  cv::minMaxLoc(depth_map, &min_val, &max_val);
  depth_map.convertTo(depth8U, CV_8UC1, 255 / (max_val - min_val));
  depth_im = depth8U;
}

/* ************************************************************************** */
Eigen::Vector3d StereoDepthSensor::getCentroid(const cv::Mat &depth_map) const {
  Eigen::Vector3d total{0, 0, 0};
  double num_points = 0;
  double cx = stereo_cam_->cx;
  double cy = stereo_cam_->cy;
  double f = stereo_cam_->f;

  for (int u = 0; u < depth_map.cols; u++) {
    for (int v = 0; v < depth_map.rows; v++) {
      double Z = depth_map.at<float>(v, u);
      if (Z == 0) continue;
      double X = (u - cx) * Z / f;
      double Y = (v - cy) * Z / f;

      total += (Eigen::Vector3d(3) << X, Y, Z).finished();
      num_points++;
    }
  }

  return (total / num_points);
}

/* ************************************************************************** */
void StereoDepthSensor::getCentroidImage(const cv::Mat &depth_im,
                                         const Eigen::Vector3d &centroid,
                                         cv::Mat &centroid_im) const {
  cv::cvtColor(depth_im, centroid_im, cv::COLOR_GRAY2BGR);
  cv::circle(
      centroid_im,
      cv::Point(centroid(0) / centroid(2) * stereo_cam_->f + stereo_cam_->cx,
                centroid(1) / centroid(2) * stereo_cam_->f + stereo_cam_->cy),
      9, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
}

/* ************************************************************************** */
void StereoDepthSensor::getCentroidImage(const Eigen::Vector3d &centroid,
                                         cv::Mat &centroid_im) const {
  cv::cvtColor(centroid_im, centroid_im, cv::COLOR_GRAY2BGR);
  cv::circle(
      centroid_im,
      cv::Point(centroid(0) / centroid(2) * stereo_cam_->f + stereo_cam_->cx,
                centroid(1) / centroid(2) * stereo_cam_->f + stereo_cam_->cy),
      9, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
}

/* ************************************************************************** */
void StereoDepthSensor::getDisparityImage(const cv::Mat &left,
                                          const cv::Mat &right,
                                          cv::Mat &disp_im) const {
  // tmp data storage containers
  cv::Mat disparity16S = cv::Mat(left.rows, left.cols, CV_16S);
  cv::Mat disparity8U = cv::Mat(left.rows, left.cols, CV_8UC1);

  this->getDisparity(left, right, &disparity16S);
  double min_val, max_val;
  cv::minMaxLoc(disparity16S, &min_val, &max_val);
  disparity16S.convertTo(disparity8U, CV_8UC1, 255 / (max_val - min_val));
  disp_im = disparity8U;
}

/* ************************************************************************** */
void StereoDepthSensor::getDisparityImage(const cv::Mat &disparity,
                                          cv::Mat &disp_im) const {
  cv::Mat disparity8U = cv::Mat(disparity.rows, disparity.cols, CV_8UC1);
  double min_val, max_val;
  cv::minMaxLoc(disparity, &min_val, &max_val);
  disparity.convertTo(disparity8U, CV_8UC1, 255 / (max_val - min_val));
  disp_im = disparity8U;
}

void StereoDepthSensor::maskByDepth(const double max_depth,
                                    const StereoFrame &frame,
                                    DepthFrame &depth_frame) const {
  // Transform depth threshold from meters to pixel disparity
  double max_disp = stereo_cam_->tx * stereo_cam_->f / max_depth;
  cv::Mat disparity_map;
  this->getDisparity(frame.img_left, frame.img_right, &disparity_map);
  cv::threshold(disparity_map, depth_frame.disp_map, max_disp * 16.0, 255,
                cv::THRESH_TOZERO);
  this->getDisparityImage(depth_frame.disp_map, depth_frame.disp_img);
}

/* ************************************************************************** */
void StereoDepthSensor::maskImageDepthThresh(const double min_depth,
                                             const double max_depth,
                                             const StereoFrame &frame,
                                             StereoFrame &out_frame) const {
  // Calculate depth map, iterate, keep only: min <= value <= max
  // double min_disp = stereo_cam_->tx * stereo_cam_->f / min_depth;
  double max_disp = stereo_cam_->tx * stereo_cam_->f / max_depth;
  std::cout << "max_disp = " << max_disp << std::endl
            << "max_disp * 16.0 = " << max_disp * 16.0 << std::endl;
  cv::Mat disparity_map, depth_map;
  this->getDisparity(frame.img_left, frame.img_right, &disparity_map);
  this->getDepthMap(disparity_map, &depth_map);
  cv::Mat depth_mask = cv::Mat::zeros(depth_map.rows, depth_map.cols, CV_8UC1);

  // cv::threshold(disparity_map, out_frame.img_left, min_disp, 255,
  //               cv::THRESH_TOZERO);
  cv::Mat thresh_disparity_map;
  cv::threshold(disparity_map, thresh_disparity_map, max_disp * 16.0, 255,
                cv::THRESH_TOZERO);
  this->getDisparityImage(disparity_map, out_frame.img_right);
  this->getDisparityImage(thresh_disparity_map, out_frame.img_left);
}

/* ************************************************************************** */
void StereoDepthSensor::fillDepthFrame(const StereoFrame &frame,
                                       DepthFrame &depth_frame) const {
  this->getDisparity(frame.img_left, frame.img_right, &(depth_frame.disp_map));
  this->getDisparityImage(depth_frame.disp_map, depth_frame.disp_img);
  this->getDepthMap(depth_frame.disp_map, &(depth_frame.depth_map));
  this->getDepthMapImage(depth_frame.depth_map, depth_frame.depth_img);
  depth_frame.centroid = this->getCentroid(depth_frame.depth_map);
}

/* ************************************************************************** */
void StereoDepthSensor::fillDepthFrame(const StereoFrame &frame,
                                       DepthFrame &depth_frame,
                                       const double max_depth) const {
  this->maskByDepth(max_depth, frame, depth_frame);  // fills disp_{map,img}
  this->getDepthMap(depth_frame.disp_map, &(depth_frame.depth_map));
  this->getDepthMapImage(depth_frame.depth_map, depth_frame.depth_img);
  depth_frame.centroid = this->getCentroid(depth_frame.depth_map);
}

/* ************************************************************************** */
Eigen::Vector4d StereoDepthSensor::getTargetMasks(
    const StereoFrame &frame, const Eigen::Vector3d &centroid,
    const double target_dim, cv::Mat &mask_left, cv::Mat &mask_right) const {
  mask_left = cv::Mat::zeros(frame.img_left.size(), CV_8U);
  mask_right = cv::Mat::zeros(frame.img_right.size(), CV_8U);

  double X = centroid(0);
  double Y = centroid(1);
  double Z = centroid(2);
  double s = target_dim / 2;

  // Quick bounds check to prevent negative numbers
  double sqr_y = (stereo_cam_->f / Z) * (Y - s) + stereo_cam_->cy;
  sqr_y = (sqr_y >= 0) ? sqr_y : 0.0;
  double sqr_x_l = (stereo_cam_->f / Z) * (X - s) + stereo_cam_->cx;
  sqr_x_l = (sqr_x_l >= 0) ? sqr_x_l : 0.0;
  double sqr_x_r = sqr_x_l - ((stereo_cam_->tx * stereo_cam_->f) / Z);
  sqr_x_r = (sqr_x_r >= 0) ? sqr_x_r : 0.0;
  double dim_px = (target_dim / Z) * stereo_cam_->f;

  // std::cout << "Calculated ROI dimensions: " << std::endl
  //           << " - sqr left:  " << sqr_x_l << std::endl
  //           << " - sqr right: " << sqr_x_r << std::endl
  //           << " - sqr y:     " << sqr_y << std::endl
  //           << " - dim_px:    " << dim_px << std::endl;

  cv::Mat roi_left(mask_left,
                   cv::Rect(int(sqr_x_l), int(sqr_y), dim_px, dim_px));
  cv::Mat roi_right(mask_right,
                    cv::Rect(int(sqr_x_r), int(sqr_y), dim_px, dim_px));
  roi_left = cv::Scalar(255);
  roi_right = cv::Scalar(255);

  return (Eigen::Vector4d(4) << sqr_x_l, sqr_x_r, sqr_y, dim_px).finished();
}

}  // namespace sph
