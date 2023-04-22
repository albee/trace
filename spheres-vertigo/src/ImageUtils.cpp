/**
 * @file ImageUtils.cpp
 * @brief Utility functions for interacting with the VERTIGO stereo images.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/ImageUtils.h"

namespace sph {

/* ************************************************************************** */
void ShowStereoPair(const std::string &title, StereoFrame *f,
                    const bool pause) {
  if (f->processed) {
    cv::drawMatches(f->img_left, f->info.keypts_left, f->img_right,
                    f->info.keypts_right, f->info.matches, f->img_matches,
                    CV_RGB(0, 255, 0), CV_RGB(255, 0, 0));
    cv::imshow(title, f->img_matches);
    if (pause) cv::waitKey(0);
  } else {
    std::cout << "StereoFrame" << f->id << " NOT PROCESSED!" << std::endl;
  }
}

/* ************************************************************************* */
void DrawMask(const cv::Mat &img, cv::Mat *out_img, const double x,
              const double y, const double s) {
  cv::cvtColor(img, *out_img, cv::COLOR_GRAY2BGR);
  cv::rectangle(*out_img, cv::Rect(x, y, s, s), cv::Scalar(255, 0, 0));
}

/* ************************************************************************* */
void ShowMask(const std::string title, const cv::Mat &img,
              const Eigen::Vector4d &mask, const bool left) {
  cv::Mat masked_img;
  DrawMask(img, &masked_img, left ? mask(0) : mask(1), mask(2), mask(3));
  cv::imshow(title, masked_img);
}

// Get the depth feature masks to speed up feature detection
/* ************************************************************************* */
void getDepthFeatureMasks(const cv::Mat &disparityImage,
                          const cv::Mat &depthMap,
                          const Eigen::Vector3d &centroid,
                          const sph::StereoCamera &stereoCamera,
                          const double minDepth, const double maxDepth,
                          cv::Rect &leftBoundingBox, cv::Rect &rightBoundingBox,
                          cv::Mat &leftDepthFeatureMask,
                          cv::Mat &rightDepthFeatureMask,
                          double *minSearchDepth, double *maxSearchDepth) {
  // Get an expanded bounding box around the foreground in the left camera frame
  cv::Mat foregroundPoints, disparityUnsigned;
  disparityImage.convertTo(disparityUnsigned, CV_8U, 1.0, 0.0);
  cv::findNonZero(disparityUnsigned, foregroundPoints);
  cv::Rect leftBoundingBoxTight = cv::boundingRect(foregroundPoints);
  leftBoundingBox = cv::Rect(
      leftBoundingBoxTight.x - 50, leftBoundingBoxTight.y - 20,
      leftBoundingBoxTight.width + 100, leftBoundingBoxTight.height + 40);

  // Calculate the dimensions of the bounding box in the right frame assuming
  int disparityShift = (int)(stereoCamera.f * stereoCamera.tx / centroid[2]);
  rightBoundingBox =
      cv::Rect(leftBoundingBox.x - disparityShift, leftBoundingBox.y,
               leftBoundingBox.width, leftBoundingBox.height);

  // Make sure bounding box is within image
  leftBoundingBox.x = std::max(0, leftBoundingBox.x);
  leftBoundingBox.y = std::max(0, leftBoundingBox.y);
  leftBoundingBox.width =
      std::min(stereoCamera.width - leftBoundingBox.x, leftBoundingBox.width);
  leftBoundingBox.height =
      std::min(stereoCamera.height - leftBoundingBox.y, leftBoundingBox.height);
  rightBoundingBox.x = std::max(0, rightBoundingBox.x);
  rightBoundingBox.y = std::max(0, rightBoundingBox.y);
  rightBoundingBox.width =
      std::min(stereoCamera.width - rightBoundingBox.x, rightBoundingBox.width);
  rightBoundingBox.height = std::min(stereoCamera.height - rightBoundingBox.y,
                                     rightBoundingBox.height);

  // Get the left and right depth feature masks
  leftDepthFeatureMask =
      cv::Mat::zeros(stereoCamera.height, stereoCamera.width, CV_8U);
  rightDepthFeatureMask =
      cv::Mat::zeros(stereoCamera.height, stereoCamera.width, CV_8U);
  cv::Mat leftSearchRegion(leftDepthFeatureMask, leftBoundingBox);
  cv::Mat rightSearchRegion(rightDepthFeatureMask, rightBoundingBox);
  leftSearchRegion = cv::Scalar(255);
  rightSearchRegion = cv::Scalar(255);

  // Get the minimum and maximum feature search depths using the depth map
  cv::Mat depthMapNoZeros = depthMap;
  depthMapNoZeros.setTo(centroid[2], depthMap <= minDepth);
  cv::minMaxLoc(depthMapNoZeros, minSearchDepth, maxSearchDepth);
  *minSearchDepth = std::max(minDepth, *minSearchDepth - 0.1);
  *maxSearchDepth = std::min(maxDepth, *maxSearchDepth + 0.1);

  /*
  std::cout << "Left bounding box [x,y,width,height]: [" << leftBoundingBox.x
            << "," << leftBoundingBox.y << "," << leftBoundingBox.width << ","
            << leftBoundingBox.height
            << "], Right bounding box [x,y,width,height]: ["
            << rightBoundingBox.x << "," << rightBoundingBox.y << ","
            << rightBoundingBox.width << "," << rightBoundingBox.height << "]"
            << std::endl;

  std::cout << "Min search depth: " << *minSearchDepth
            << "m, Max search depth: " << *maxSearchDepth << "m" << std::endl
            << std::flush;
  */
}

}  // namespace sph
