/**
 * @file FrontEnd.cpp
 * @brief Class for dealing with sensor measurements.
 * @date August 01, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/FrontEnd.h"

namespace sph {

/* ************************************************************************** */
FrontEnd::FrontEnd(const FrontEndParams &params) : params_(params) {
  // Get calibration parameters to build stereo camera.
  rectifier_ = std::make_shared<Rectifier>(params_.dataset_path +
                                           "OpticsMount_0_Images/");
  rectifier_->calcRectificationMaps();
  cam_ = rectifier_->createStereoCam();

  // Compute constants.
  min_disparity_ = cam_->tx * cam_->f / params_.max_depth;
  max_disparity_ = cam_->tx * cam_->f / params_.min_depth;
  min_disparity_ = (min_disparity_ >= 0) ? min_disparity_ : 0;
  max_disparity_ = (max_disparity_ <= 255) ? max_disparity_ : 255;

  // Set up the depth sensor.
  depth_ = std::make_shared<StereoDepthSensor>(cam_, params_.num_disp,
                                               params_.block_size);

  // Set up the feature handler.
  detector_ =
      cv::xfeatures2d::SURF::create(params_.hessianThresh,  // hessianThreshold
                                    4,                      // nOctaves=4
                                    4,                      // nOctaveLayers=2
                                    true,                   // extended=true
                                    false);                 // upright=false
  extractor_ =
      cv::xfeatures2d::SURF::create(params_.hessianThresh,  // hessianThreshold
                                    4,                      // nOctaves=4
                                    4,                      // nOctaveLayers=2
                                    true,                   // extended=true
                                    false);                 // upright=false
  matcher_ = new cv::BFMatcher();
  fh_ = std::make_shared<FeatureHandler>(params_.detectorExtractorMatcherTypes,
                                         detector_, extractor_, matcher_,
                                         cam_.get());
  fh_->setMaxDescriptorDistance(0.5);
  fh_->setMinMaxDepth(params_.min_depth, params_.max_depth);

  // Set up the visual odometer.
  vo_ = std::make_shared<VisualOdometer>(cam_.get(), fh_.get());

  // Get pose of stereo camera in body frame.
  T_BC_ = sph::Pose3D(params_.T_BC);
}

/* ************************************************************************** */
FrontEnd::~FrontEnd() {}

/* ************************************************************************** */
void FrontEnd::ProcessImage(const sph::StereoFrame &frame,
                            omsci::StereoFrame *processed_frame) const {
  // Compute depth and disparity maps.
  sph::DepthFrame depth_frame;
  depth_frame.disp_img =
      cv::Mat(frame.img_left.rows, frame.img_left.cols, CV_16S);
  depth_->fillDepthFrame(frame, depth_frame);

  // Threshold on valid pixels.
  cv::Mat disp_map_thresh;
  depth_frame.disp_map.convertTo(disp_map_thresh, CV_32F);
  std::for_each(disp_map_thresh.begin<float>(), disp_map_thresh.end<float>(),
                [](float &pixel) { pixel = pixel / 16.0; });
  cv::threshold(disp_map_thresh, disp_map_thresh, min_disparity_, 255,
                cv::THRESH_TOZERO);
  cv::threshold(disp_map_thresh, disp_map_thresh, max_disparity_, 255,
                cv::THRESH_TOZERO_INV);

  // Mask around valid pixels. TODO(tonioteran) Refactor.
  cv::Rect leftBoundingBox, rightBoundingBox;
  cv::Mat leftDepthFeatureMask, rightDepthFeatureMask;
  double minSearchDepth, maxSearchDepth;

  sph::getDepthFeatureMasks(
      disp_map_thresh, depth_frame.depth_map, depth_frame.centroid, *cam_,
      params_.min_depth, params_.max_depth, leftBoundingBox, rightBoundingBox,
      leftDepthFeatureMask, rightDepthFeatureMask, &minSearchDepth,
      &maxSearchDepth);

  // Create the processed stereo frame.
  fh_->setCustomStereoFeatureMask(leftDepthFeatureMask, rightDepthFeatureMask);
  fh_->setMinMaxDepth(minSearchDepth, maxSearchDepth);
  fh_->getStereoFrame(frame.img_left, frame.img_right, *processed_frame);
}

/* ************************************************************************** */
void FrontEnd::MatchPair(const omsci::StereoFrame &processed_frame_i,
                         const omsci::StereoFrame &processed_frame_j,
                         omsci::StereoFrame *inlier_frame_i,
                         omsci::StereoFrame *inlier_frame_j,
                         Pose3D *poseCitoCj) const {
  vo_->calculateMotionMaximumLikelihood(processed_frame_i, processed_frame_j,
                                        *inlier_frame_i, *inlier_frame_j,
                                        *poseCitoCj);
}

/* ************************************************************************** */
void FrontEnd::DrawMatched(const sph::StereoFrame &frame_j,
                           const omsci::StereoFrame &processed_frame_j,
                           const omsci::StereoFrame &inlier_frame_i,
                           const omsci::StereoFrame &inlier_frame_j,
                           cv::Mat *out) const {
  fh_->showStereoFeatures(frame_j.img_left, frame_j.img_right,
                          processed_frame_j, *out);
  fh_->showMatchedStereoFrames(
      frame_j.img_left, frame_j.img_right, inlier_frame_i, inlier_frame_j, *out,
      CV_RGB(0x00, 0xFF, 0x00), CV_RGB(0x00, 0xFF, 0x00));
}

/* ************************************************************************** */
void FrontEnd::OnlyDrawMatches(const sph::StereoFrame &frame_j,
                               const omsci::StereoFrame &processed_frame_j,
                               const omsci::StereoFrame &inlier_frame_i,
                               const omsci::StereoFrame &inlier_frame_j,
                               const cv::Scalar &rgbLR, const cv::Scalar &rgbAB,
                               cv::Mat *out) const {
  fh_->showMatchedStereoFrames(frame_j.img_left, frame_j.img_right,
                               inlier_frame_i, inlier_frame_j, *out, rgbLR,
                               rgbAB);
}

/* ************************************************************************** */
void FrontEnd::GeometricFrame(const omsci::StereoFrame &inlier_frame,
                              sph::Pose3D *T_GB) const {
  // Get translation from camera to triangulated features' centroid.
  Eigen::Vector3d t_GwrtC_C = fh_->getCentroid(inlier_frame);
  Eigen::Vector3d t_BwrtG_G = -(T_BC_.R * t_GwrtC_C + T_BC_.t);
  *T_GB = sph::Pose3D(Eigen::Matrix3d::Identity(), t_BwrtG_G);
}

}  // namespace sph
