/**
 * @file ImageUtils.h
 * @brief Utility functions for interacting with the VERTIGO stereo images.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_IMAGEUTILS_H_
#define SPH_VERTIGO_IMAGEUTILS_H_

#include <string>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @brief Draw and show matches for a processed stereo frame.
 * @param[in] title Name of the window.
 * @param[in/out] f Processed stereo frame with keypoints and matches.
 * @param[in] pause Whether to call OpenCV's `waitKey(0)` function.
 *
 * The joint stereo pair with drawn matches get stored in `f`.
 */
void ShowStereoPair(const std::string &title, StereoFrame *f,
                    const bool pause = true);

//! Draws rectangle using specified parameters on top of img
void DrawMask(const cv::Mat &img, cv::Mat *out_img, const double x,
              const double y, const double s);

//! Custom imshow to show rectangular ROI mask on original image
void ShowMask(const std::string title, const cv::Mat &img,
              const Eigen::Vector4d &mask, const bool left = true);

// Get the depth feature masks to speed up feature detection
void getDepthFeatureMasks(const cv::Mat &disparityImage,
                          const cv::Mat &depthMap,
                          const Eigen::Vector3d &centroid,
                          const sph::StereoCamera &stereoCamera,
                          const double minDepth, const double maxDepth,
                          cv::Rect &leftBoundingBox, cv::Rect &rightBoundingBox,
                          cv::Mat &leftDepthFeatureMask,
                          cv::Mat &rightDepthFeatureMask,
                          double *minSearchDepth, double *maxSearchDepth);

}  // namespace sph

#include "sph-vertigo/DataUtils.tpp"

#endif  // SPH_VERTIGO_IMAGEUTILS_H_
