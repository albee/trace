// #include <math.h>  conflicts with cmath.h!!!
#include <algorithm>
#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include "sph-vertigo/FeatureHandler.h"

namespace sph {

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

// Constructor stereo camera; types of detector, extractor and matcher are
// listed in types.h
FeatureHandler::FeatureHandler(int* detectorExtractorMatcherTypes,
                               cv::FeatureDetector* featureDetector,
                               cv::DescriptorExtractor* descriptorExtractor,
                               cv::DescriptorMatcher* descriptorMatcher,
                               StereoCamera* camera)
    : featureDetector_(featureDetector),
      descriptorExtractor_(descriptorExtractor),
      descriptorMatcher_(descriptorMatcher),
      featureDetectorType_(detectorExtractorMatcherTypes[0]),
      descriptorExtractorType_(detectorExtractorMatcherTypes[1]),
      descriptorMatcherType_(detectorExtractorMatcherTypes[2]),
      stereoCamera_(camera) {
  triangulatorMaxVerticalOffset_ = 2;  // set maximum vertical offset [px]
  setMinMaxDepth(
      0.001, 100.0);  // set minimum and maximum depth of features to keep [m]
  maxDescriptorDistance_ = 0.9;  // default maximum descriptor distance
  sortingOn_ = true;             // whether or not to use v coordinate sorting
  useCustomFeatureMask_ =
      false;  // by default do no use custom feature mask unless added

  totalTimeDetectFunc_ = 0.0;   // no time spent detecting yet		 [ms]
  totalTimeExtractFunc_ = 0.0;  // no time spent extracting yet 	 [ms]
  totalTimeMatchFunc_ = 0.0;    // no time spent matching yet   	 [ms]
  totalTimeTrianFunc_ = 0.0;    // no time spent triangulating yet   [ms]
  totalNumDetectFunc_ = 0;      // no detect executions yet
  totalNumExtractFunc_ = 0;     // no extract executions yet
  totalNumMatchFunc_ = 0;       // no match executions yet
  totalNumTrianFunc_ = 0;       // no triangulate executions yet

  //	absoluteOrientationInlierThreshold_ = 0.005;		// inlier
  // tolerance
  //[m] 	absoluteOrientationRansacMaxIterations_ = 200;	 	//
  // maximum iterations for absolute orientation RANSAC
  //	absoluteOrientationMininmumInliers_ = 4; 			//
  // minimum number of inliers for absolute orientation
  // aboluteOrientationMinInterpointDist_ =
  // 0.001;		// minimum distance between any two RANSAC points
  // selected for absolute orientation [m]

  std::cout << "Stereo FeatureHandler created." << std::endl;
}

// Destructor
FeatureHandler::~FeatureHandler() {}

// Get keypoints and descriptors for all features
void FeatureHandler::getFeatures(const cv::Mat& image,
                                 std::vector<cv::KeyPoint>& keypoints,
                                 cv::Mat& descriptors, const cv::Mat& mask) {
  // Initialize unsorted keypoints and descriptors
  std::vector<cv::KeyPoint> keypointsUnsorted;
  cv::Mat descriptorsUnsorted;

  // Detect keypoints
  // featureDetector_->detect(image, keypointsUnsorted, mask);
  SURFhandler_->detect(image, keypointsUnsorted, mask);

  totalNumDetectFunc_++;

  // TODO: in the stereo case, can mask features from being extracted that do
  // not have any features in vertical tolerance on the other side of the
  // image (might offer minor speedup)

  // Extract descriptors
  // descriptorExtractor_->compute(image, keypointsUnsorted,
  // descriptorsUnsorted);
  SURFhandler_->compute(image, keypointsUnsorted, descriptorsUnsorted);

  // Erase any previous elements
  keypoints.clear();
  descriptors = cv::Mat(descriptorsUnsorted.rows, descriptorsUnsorted.cols,
                        descriptorsUnsorted.type());

  // If sorting is on, sort the keypoints by v coordinate
  if (sortingOn_) {
    // Create a standard sequence of indices in numerical order
    std::vector<size_t> keypointIndices(keypointsUnsorted.size());
    for (size_t i = 0; i < keypointIndices.size(); i++) {
      keypointIndices[i] = i;
    }

    // Sort indices by v coordinate (ascending)
    std::sort(keypointIndices.begin(), keypointIndices.end(),
              omsci::compareKeypointIndexY(keypointsUnsorted));

    // Populate sorted keypoints and descriptors
    for (size_t i = 0; i < keypointIndices.size(); i++) {
      keypoints.push_back(keypointsUnsorted[keypointIndices[i]]);
      descriptorsUnsorted.row(keypointIndices[i]).copyTo(descriptors.row(i));
    }

  } else {
    keypoints = keypointsUnsorted;
    descriptors = descriptorsUnsorted.clone();
  }

  totalNumExtractFunc_++;
}

// Get a lookup table that is height+1 elements long and at lookupTable[row]
// contains the index of the first keypoint >= row px (sorted by v coordinate);
// the last element is the last feature index + 1
void FeatureHandler::getRowLookupTable(
    const std::vector<cv::KeyPoint>& keypoints, int imageHeight,
    std::vector<int>& lookupTable) {
  // Clear lookup table and resize to necessary dimension
  lookupTable.clear();
  lookupTable.resize(imageHeight + 1);

  // Create lookup table
  int keypointIndex = 0;
  for (int row = 0; row < imageHeight + 1; row++) {
    lookupTable[row] = std::min((int)keypoints.size(), keypointIndex);

    // While on the same row, continue to increment through the keypoints
    while (keypointIndex < (int)keypoints.size() &&
           std::floor(keypoints[keypointIndex].pt.y) == (float)row) {
      keypointIndex++;
    }
  }
}

// Match descriptors1 and descriptors2 and return DMatch values. When mask[i,j]
// = 0, it means that descriptors1[i] cannot be matched with descriptors2[j]
void FeatureHandler::matchFeatures(const cv::Mat& descriptors1,
                                   const cv::Mat& descriptors2,
                                   std::vector<cv::DMatch>& matches,
                                   const cv::Mat& mask) {
  // Match (query) descriptors 1 against (training) descriptors 2
  std::vector<cv::DMatch> matchesAll;

  descriptorMatcher_->match(descriptors1, descriptors2, matchesAll, mask);

  // Only keep matches within distance tolerance
  matches.clear();
  for (int i = 0; i < (int)matchesAll.size(); i++) {
    //		std::cout << matchesAll[i].distance << std::endl;

    if ((double)matchesAll[i].distance <= maxDescriptorDistance_) {
      matches.push_back(matchesAll[i]);
    }
  }
}

// Get the average time that it takes to detect, extract, and match features
void FeatureHandler::getAverageTimes(double& detectTime, double& extractTime,
                                     double& matchTime,
                                     double& triangulateTime) {
  detectTime = totalTimeDetectFunc_ / ((double)totalNumDetectFunc_);
  extractTime = totalTimeExtractFunc_ / ((double)totalNumExtractFunc_);
  matchTime = totalTimeMatchFunc_ / ((double)totalNumMatchFunc_);
  triangulateTime = totalTimeTrianFunc_ / ((double)totalNumTrianFunc_);
}

// Set the minimum and maximum depth of features to keep
void FeatureHandler::setMinMaxDepth(double min, double max) {
  // Minimum and maximum depth of features to keep [m]
  minDepth_ = min;
  maxDepth_ = max;

  // Minimum / maximum disparity based on maximum / minimum depth [px]
  minDisparity_ = stereoCamera_->tx * stereoCamera_->f / maxDepth_;
  maxDisparity_ = stereoCamera_->tx * stereoCamera_->f / minDepth_;

  if (!useCustomFeatureMask_) {
    // Create masks to reduce the search area when getting stereo frames
    leftFeatureMask_ =
        cv::Mat::zeros(stereoCamera_->height, stereoCamera_->width, CV_8U);
    rightFeatureMask_ =
        cv::Mat::zeros(stereoCamera_->height, stereoCamera_->width, CV_8U);

    cv::Mat leftSearchRegion(
        leftFeatureMask_,
        cv::Rect((int)floor(minDisparity_), 0,
                 stereoCamera_->width - ((int)floor(minDisparity_)),
                 stereoCamera_->height));
    leftSearchRegion = cv::Scalar(255);

    cv::Mat rightSearchRegion(
        rightFeatureMask_,
        cv::Rect(0, 0, stereoCamera_->width - ((int)floor(minDisparity_)),
                 stereoCamera_->height));
    rightSearchRegion = cv::Scalar(255);
  }
}

// Set the maximum distance between features that is considered a match
void FeatureHandler::setMaxDescriptorDistance(double maxDist) {
  maxDescriptorDistance_ = maxDist;
}

// Set a custom feature mask for stereo images
void FeatureHandler::setCustomStereoFeatureMask(
    const cv::Mat& leftFeatureMask, const cv::Mat& rightFeatureMask) {
  leftFeatureMask_ = leftFeatureMask;
  rightFeatureMask_ = rightFeatureMask;
  useCustomFeatureMask_ = true;
}

// Get the stereo frame information from an image pair
void FeatureHandler::getStereoFrame(const cv::Mat& leftImage,
                                    const cv::Mat& rightImage,
                                    omsci::StereoFrame& frame) {
  // Reset the contents of stereo frame
  frame = omsci::StereoFrame();

  // Get features
  getFeatures(leftImage, frame.leftKeypoints, frame.leftDescriptors,
              leftFeatureMask_);
  getFeatures(rightImage, frame.rightKeypoints, frame.rightDescriptors,
              rightFeatureMask_);

  // If sorting is on
  if (sortingOn_) {
    // Create a blank mask full of zeros
    cv::Mat matchMask(frame.leftDescriptors.rows, frame.rightDescriptors.rows,
                      CV_8U, cv::Scalar(0));

    // Get the lookup tables to speed up matching (leftRowLookupTable not used)
    //		getRowLookupTable(frame.leftKeypoints,  leftImage.rows,
    // frame.leftRowLookupTable);
    getRowLookupTable(frame.rightKeypoints, rightImage.rows,
                      frame.rightRowLookupTable);

    // Create a mask so that only features out of the right descriptors within
    // vertical tolerance will be attempted to be matched. leftDescritorss[i]
    // can be matched with rightDescriptors[j] only if mask.at<uchar>(i,j) is
    // non-zero.
    // http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#descriptormatcher-match

    // Loop through all left descriptors, and set all viable matching indices to
    // 255
    for (int i = 0; i < (int)frame.leftDescriptors.rows; i++) {
      int leftCoordV = (int)std::round(frame.leftKeypoints[i].pt.y);
      int minRightCoordV =
          std::max(0, leftCoordV - triangulatorMaxVerticalOffset_);
      int maxRightCoordV = std::min(
          rightImage.rows + 1, leftCoordV + triangulatorMaxVerticalOffset_ + 1);

      for (int j = frame.rightRowLookupTable[minRightCoordV];
           j < frame.rightRowLookupTable[maxRightCoordV]; j++) {
        matchMask.at<uchar>(i, j) = 255;
      }
    }

    // Get left to right matches
    matchFeatures(frame.leftDescriptors, frame.rightDescriptors,
                  frame.leftRightMatches, matchMask);

  } else {
    // Get left to right matches
    matchFeatures(frame.leftDescriptors, frame.rightDescriptors,
                  frame.leftRightMatches);
  }

  //	std::stringstream filename1, filename2;
  //	filename2 << "/home/Results/matchMask.bmp";
  //	cv::imwrite(filename2.str(), matchMask);

  totalNumMatchFunc_++;

  // Get number of matches before triangulation
  int numMatchesBeforeTriangulate = frame.leftRightMatches.size();

  // Get 3D points, and eliminate those not within v coordinate or depth
  // tolerance
  triangulate(frame);

  std::cout
      << "StereoFrame Features [L, R / LRMatched / LRTriangulated] [time]: ["
      << frame.leftKeypoints.size() << ", " << frame.rightKeypoints.size()
      << " / " << numMatchesBeforeTriangulate << " / "
      << frame.leftRightMatches.size() << "]" << std::endl;
}

// Create an outImage with all detected features in the left and right images of
// a stereo frame
void FeatureHandler::showStereoFeatures(const cv::Mat& leftImage,
                                        const cv::Mat& rightImage,
                                        const omsci::StereoFrame& frame,
                                        cv::Mat& outImage, cv::Scalar rgbLR) {
  cv::Mat outImageLeft, outImageRight;
  cv::Size size(leftImage.cols + rightImage.cols,
                MAX(leftImage.rows, rightImage.rows));

  bool isOutImageEmpty = false;
  if (!outImage.data) {
    // If outImage is empty, then create it
    isOutImageEmpty = true;
    outImage.create(size, CV_MAKETYPE(leftImage.depth(), 3));
  }

  outImageLeft = outImage(cv::Rect(0, 0, leftImage.cols, leftImage.rows));
  outImageRight =
      outImage(cv::Rect(leftImage.cols, 0, rightImage.cols, rightImage.rows));

  // If outImage is empty, put the images in it
  if (isOutImageEmpty) {
    cvtColor(leftImage, outImageLeft, cv::COLOR_GRAY2BGR);
    cvtColor(rightImage, outImageRight, cv::COLOR_GRAY2BGR);
  }

  // Add the left points
  for (int i = 0; i < (int)frame.leftKeypoints.size(); i++) {
    cv::circle(
        outImageLeft,
        cv::Point(frame.leftKeypoints[i].pt.x, frame.leftKeypoints[i].pt.y), 1,
        rgbLR, -1, cv::LINE_AA);
  }

  // Add the right points
  for (int i = 0; i < (int)frame.rightKeypoints.size(); i++) {
    cv::circle(
        outImageRight,
        cv::Point(frame.rightKeypoints[i].pt.x, frame.rightKeypoints[i].pt.y),
        1, rgbLR, -1, cv::LINE_AA);
  }
}

// Create an outImage with all detected features in the left and right images of
// a stereo frame
void FeatureHandler::showStereoFeatures(const cv::Mat& leftImage,
                                        const cv::Mat& rightImage,
                                        const omsci::StereoFrame& frame,
                                        cv::Mat& outImage) {
  showStereoFeatures(leftImage, rightImage, frame, outImage,
                     CV_RGB(0x00, 0x00, 0xFF));
}

// Create an outImage of a stereo frame with the matched features shown,
// choosing the display color
void FeatureHandler::showStereoFrame(const cv::Mat& leftImage,
                                     const cv::Mat& rightImage,
                                     const omsci::StereoFrame& frame,
                                     cv::Mat& outImage, cv::Scalar rgbLR) {
  cv::Mat outImageLeft, outImageRight;
  cv::Size size(leftImage.cols + rightImage.cols,
                MAX(leftImage.rows, rightImage.rows));

  bool isOutImageEmpty = false;
  if (!outImage.data) {
    // If outImage is empty, then create it
    isOutImageEmpty = true;
    outImage.create(size, CV_MAKETYPE(leftImage.depth(), 3));
  }

  outImageLeft = outImage(cv::Rect(0, 0, leftImage.cols, leftImage.rows));
  outImageRight =
      outImage(cv::Rect(leftImage.cols, 0, rightImage.cols, rightImage.rows));

  // If outImage is empty, put the images in it
  if (isOutImageEmpty) {
    cvtColor(leftImage, outImageLeft, cv::COLOR_GRAY2BGR);
    cvtColor(rightImage, outImageRight, cv::COLOR_GRAY2BGR);
  }

  // Add the points and the lines
  for (int i = 0; i < (int)frame.leftRightMatches.size(); i++) {
    int leftInd = frame.leftRightMatches[i].queryIdx;
    int rightInd = frame.leftRightMatches[i].trainIdx;
    cv::circle(outImageRight,
               cv::Point(frame.leftKeypoints[leftInd].pt.x,
                         frame.leftKeypoints[leftInd].pt.y),
               1, rgbLR, -1, cv::LINE_AA);
    cv::circle(outImageRight,
               cv::Point(frame.rightKeypoints[rightInd].pt.x,
                         frame.rightKeypoints[rightInd].pt.y),
               2, rgbLR, -1, cv::LINE_AA);
    cv::line(outImageRight,
             cv::Point(frame.leftKeypoints[leftInd].pt.x,
                       frame.leftKeypoints[leftInd].pt.y),
             cv::Point(frame.rightKeypoints[rightInd].pt.x,
                       frame.rightKeypoints[rightInd].pt.y),
             rgbLR);
  }

  // Create a translucent backdrop to place text on
  double alpha = 0.4;
  cv::Mat translucentRegion = outImage(cv::Rect(0, 0, 1280, 30));
  cv::Mat backdrop(translucentRegion.size(), CV_MAKETYPE(leftImage.depth(), 3),
                   CV_RGB(0, 0, 0));
  cv::addWeighted(backdrop, alpha, translucentRegion, 1.0 - alpha, 0.0,
                  translucentRegion);

  // Annotate the image
  std::string text = "Left-to-right correspondences";
  cv::putText(outImageRight, text, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX,
              0.5, rgbLR, 1.5, cv::LINE_AA, false);
}

// Create an outImage of a stereo frame with the matched features shown
void FeatureHandler::showStereoFrame(const cv::Mat& leftImage,
                                     const cv::Mat& rightImage,
                                     const omsci::StereoFrame& frame,
                                     cv::Mat& outImage) {
  showStereoFrame(leftImage, rightImage, frame, outImage,
                  CV_RGB(0x00, 0x00, 0xFF));
}

// Create an outImage of two matched stereo frames for easy interpretation,
// choosing the display colors
void FeatureHandler::showMatchedStereoFrames(
    const cv::Mat& leftImageB, const cv::Mat& rightImageB,
    const omsci::StereoFrame& matchedFrameA,
    const omsci::StereoFrame& matchedFrameB, cv::Mat& outImage,
    cv::Scalar rgbLR, cv::Scalar rgbAB) {
  cv::Mat outImageLeft, outImageRight;
  cv::Size size(leftImageB.cols + rightImageB.cols,
                MAX(leftImageB.rows, rightImageB.rows));

  // If outImage is empty, then create it
  bool isOutImageEmpty = false;
  if (!outImage.data) {
    isOutImageEmpty = true;
    outImage.create(size, CV_MAKETYPE(leftImageB.depth(), 3));
  }

  // Partition the output image
  outImageLeft = outImage(cv::Rect(0, 0, leftImageB.cols, leftImageB.rows));
  outImageRight = outImage(
      cv::Rect(leftImageB.cols, 0, rightImageB.cols, rightImageB.rows));

  // If outImage is empty, put imageB into it
  if (isOutImageEmpty) {
    cvtColor(leftImageB, outImageLeft, cv::COLOR_GRAY2BGR);
    cvtColor(rightImageB, outImageRight, cv::COLOR_GRAY2BGR);
  }

  // Show stereo correspondences
  showStereoFrame(leftImageB, rightImageB, matchedFrameB, outImage, rgbLR);

  for (int i = 0; i < (int)matchedFrameA.leftRightMatches.size(); i++) {
    int leftIndA = matchedFrameA.leftRightMatches[i].queryIdx;
    int leftIndB = matchedFrameB.leftRightMatches[i].queryIdx;

    // Put colored dots at all feature points on the left hand side of the
    // output image Make the dots in frame B bigger than those in frame A
    cv::circle(outImageLeft,
               cv::Point(matchedFrameA.leftKeypoints[leftIndA].pt.x,
                         matchedFrameA.leftKeypoints[leftIndA].pt.y),
               1, rgbAB, -1, cv::LINE_AA);
    cv::circle(outImageLeft,
               cv::Point(matchedFrameB.leftKeypoints[leftIndB].pt.x,
                         matchedFrameB.leftKeypoints[leftIndB].pt.y),
               2, rgbAB, -1, cv::LINE_AA);

    // Draw colored lines between features and their corresponding feature in
    // the other frame
    cv::line(outImage,
             cv::Point(matchedFrameA.leftKeypoints[leftIndA].pt.x,
                       matchedFrameA.leftKeypoints[leftIndA].pt.y),
             cv::Point(matchedFrameB.leftKeypoints[leftIndB].pt.x,
                       matchedFrameB.leftKeypoints[leftIndB].pt.y),
             rgbAB);
  }

  // Annotate the image
  std::stringstream text;
  text << "Frame-to-frame correspondences ("
       << matchedFrameA.leftRightMatches.size() << ")";
  cv::putText(outImageLeft, text.str(), cv::Point(10, 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, rgbLR, 1.5, cv::LINE_AA, false);
}

// Create an outImage of two matched stereo frames for easy interpretation
void FeatureHandler::showMatchedStereoFrames(
    const cv::Mat& leftImageB, const cv::Mat& rightImageB,
    const omsci::StereoFrame& matchedFrameA,
    const omsci::StereoFrame& matchedFrameB, cv::Mat& outImage) {
  showMatchedStereoFrames(leftImageB, rightImageB, matchedFrameA, matchedFrameB,
                          outImage, CV_RGB(0x00, 0x00, 0xFF),
                          CV_RGB(0x00, 0x00, 0xFF));
}

// Merge two stereo frames
void FeatureHandler::mergeStereoFrames(omsci::StereoFrame& frameA,
                                       omsci::StereoFrame& frameB,
                                       omsci::StereoFrame& frameAB) {
  // Clear the merged frame
  frameAB.leftKeypoints.clear();
  frameAB.rightKeypoints.clear();
  frameAB.leftDescriptors = cv::Mat();
  frameAB.rightDescriptors = cv::Mat();
  frameAB.leftRightMatches.clear();
  frameAB.leftRightMatches3D.clear();
  frameAB.covLeftRightMatches3D.clear();

  // Merge keypoints
  frameAB.leftKeypoints = frameA.leftKeypoints;
  frameAB.rightKeypoints = frameA.rightKeypoints;
  frameAB.leftKeypoints.insert(frameAB.leftKeypoints.end(),
                               frameB.leftKeypoints.begin(),
                               frameB.leftKeypoints.end());
  frameAB.rightKeypoints.insert(frameAB.rightKeypoints.end(),
                                frameB.rightKeypoints.begin(),
                                frameB.rightKeypoints.end());

  // Merge descriptors
  frameA.leftDescriptors.copyTo(frameAB.leftDescriptors);
  frameA.rightDescriptors.copyTo(frameAB.rightDescriptors);
  frameAB.leftDescriptors.push_back(frameB.leftDescriptors);
  frameAB.rightDescriptors.push_back(frameB.rightDescriptors);

  // Merge 3D matches
  frameAB.leftRightMatches3D = frameA.leftRightMatches3D;
  frameAB.leftRightMatches3D.insert(frameAB.leftRightMatches3D.end(),
                                    frameB.leftRightMatches3D.begin(),
                                    frameB.leftRightMatches3D.end());

  // Merge 2D matches
  frameAB.leftRightMatches = frameA.leftRightMatches;
  size_t numLeftKeypointsA = frameA.leftKeypoints.size();
  size_t numRightKeypointsA = frameA.rightKeypoints.size();
  size_t numMatchesB = frameB.leftRightMatches.size();

  // Merge covariances
  frameAB.covLeftRightMatches3D = frameA.covLeftRightMatches3D;
  frameAB.covLeftRightMatches3D.insert(frameAB.covLeftRightMatches3D.end(),
                                       frameB.covLeftRightMatches3D.begin(),
                                       frameB.covLeftRightMatches3D.end());

  // Loop through and reassign indices in new, merged frameAB
  for (int i = 0; i < (int)numMatchesB; i++) {
    cv::DMatch mergeMatch = frameB.leftRightMatches[i];

    if (frameA.leftRightMatches.empty()) {
      mergeMatch.imgIdx = 1;
      mergeMatch.queryIdx = frameB.leftRightMatches[i].queryIdx;
      mergeMatch.trainIdx = frameB.leftRightMatches[i].trainIdx;

    } else {
      mergeMatch.imgIdx = frameA.leftRightMatches.back().imgIdx + 1;
      mergeMatch.queryIdx =
          ((int)numLeftKeypointsA) + frameB.leftRightMatches[i].queryIdx;
      mergeMatch.trainIdx =
          ((int)numRightKeypointsA) + frameB.leftRightMatches[i].trainIdx;
    }

    frameAB.leftRightMatches.push_back(mergeMatch);
  }
}

// Triangulate left and right keypoints, eliminate all matches that don't fall
// within the maximum vertical offset or between the minimum and maximum depth,
// and store the 3D points of the good matches.
void FeatureHandler::triangulate(omsci::StereoFrame& frame) {
  double uL, uR, vL, vR;  // 2D image coords in left and right frames [px]
  double x, y, z;         // 3D image coords in left camera frame [m]
  std::vector<cv::DMatch>::size_type i = 0;  // iteration variable

  // Create local variables with camera parameters
  double f = stereoCamera_->f;
  double tx = stereoCamera_->tx;
  double cx = stereoCamera_->cx;
  double cy = stereoCamera_->cy;
  Eigen::Matrix3d covS = stereoCamera_->covS;

  // While the entire array hasn't been iterated through, triangulate the points
  while (i != frame.leftRightMatches.size()) {
    // The vertical point location in the left and right images [px]
    vL = (double)frame.leftKeypoints[frame.leftRightMatches[i].queryIdx].pt.y;
    vR = (double)frame.rightKeypoints[frame.leftRightMatches[i].trainIdx].pt.y;

    // If the vertical coords aren't within tolerance, erase the match;
    // otherwise, keep going
    if (abs(vL - vR) > (double)triangulatorMaxVerticalOffset_) {
      frame.leftRightMatches.erase(frame.leftRightMatches.begin() + i);

    } else {
      // TODO: Use Q matrix from camera calibration instead of simply using the
      // focal length - this likely does not include the effects of
      // rectification
      // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20reprojectImageTo3D%28InputArray%20disparity,%20OutputArray%20_3dImage,%20InputArray%20Q,%20bool%20handleMissingValues,%20int%20ddepth%29
      // [ X Y Z W] = Q * [u v disp 1]'
      // [ X Y Z ] = [X Y Z]/W
      // Check how different the Q matrix would be if made entirely using the f
      // and tx below

      // The horizontal point location in the left and right images [px]
      uL = (double)frame.leftKeypoints[frame.leftRightMatches[i].queryIdx].pt.x;
      uR =
          (double)frame.rightKeypoints[frame.leftRightMatches[i].trainIdx].pt.x;

      // Calculate the depth [m]
      z = (tx * f) / (uL - uR);

      // If depth is out of tolerance, erase the match; otherwise, store the
      // point
      if (z < minDepth_ || z > maxDepth_) {
        frame.leftRightMatches.erase(frame.leftRightMatches.begin() + i);

      } else {
        // Calculate the coordinates [m]
        x = (uL - cx) * z / f;
        y = (vL - cy) * z / f;

        // Create and store the point
        Eigen::Vector3d featPoint3D(x, y, z);
        frame.leftRightMatches3D.push_back(featPoint3D);

        // Find the Jacobian of feature location r = [x y z]' with respect to
        // stereo image coordinate s = [uL uR vL]'
        Eigen::Matrix3d Jr_s = Eigen::Matrix3d::Zero();
        Jr_s(0, 0) =
            tx / (uL - uR) + (tx * (cx - uL)) / pow(uL - uR, 2);  // dx/duL
        Jr_s(0, 1) = -(tx * (cx - uL)) / pow(uL - uR, 2);         // dx/duR
        Jr_s(1, 0) = (tx * (cy - vL)) / pow(uL - uR, 2);          // dy/duL
        Jr_s(1, 1) = -(tx * (cy - vL)) / pow(uL - uR, 2);         // dy/duL
        Jr_s(1, 2) = tx / (uL - uR);                              // dy/dvL
        Jr_s(2, 0) = -(f * tx) / pow(uL - uR, 2);                 // dz/duL
        Jr_s(2, 1) = (f * tx) / pow(uL - uR, 2);                  // dz/duR

        // Find the covariance of the point r = [x y z]'
        frame.covLeftRightMatches3D.push_back(Jr_s * covS * Jr_s.transpose());

        // Increment the iterator to move onto the next point
        i++;
      }
    }
  }

  totalNumTrianFunc_++;
}

// Reproject a single point back onto the image
Eigen::Vector2d FeatureHandler::reproject(const Eigen::Vector3d& point3D) {
  Eigen::Vector2d pointImage;
  pointImage(0) =
      stereoCamera_->f * point3D(0) / point3D(2) + stereoCamera_->cx;
  pointImage(1) =
      stereoCamera_->f * point3D(1) / point3D(2) + stereoCamera_->cy;

  return pointImage;
}

// Match two stereo frames, returning stereo frames containing only the subset
// of leftRightMatches(3D) that also matches frame to frame
void FeatureHandler::matchStereoFrames(const omsci::StereoFrame& frameA,
                                       const omsci::StereoFrame& frameB,
                                       omsci::StereoFrame& matchedFrameA,
                                       omsci::StereoFrame& matchedFrameB) {
  // Find all matches from frameA-to-frameB; each feature needs to be described
  // by four features

  // Points, DMatches, and covariances from both frames, where the indices of
  // matching features are aligned
  std::vector<Eigen::Vector3d> points3DFrameA, points3DFrameB;
  std::vector<cv::DMatch> matchesFrameA, matchesFrameB;
  std::vector<Eigen::Matrix3d> covs3DFrameA, covs3DFrameB;

  // Copy existing matches
  std::vector<cv::DMatch> frameALeftRightMatches = frameA.leftRightMatches;
  std::vector<cv::DMatch> frameBLeftRightMatches = frameB.leftRightMatches;
  std::vector<Eigen::Vector3d> frameALeftRightMatches3D =
      frameA.leftRightMatches3D;
  std::vector<Eigen::Vector3d> frameBLeftRightMatches3D =
      frameB.leftRightMatches3D;
  std::vector<Eigen::Matrix3d> frameACovLeftRightMatches3D =
      frameA.covLeftRightMatches3D;
  std::vector<Eigen::Matrix3d> frameBCovLeftRightMatches3D =
      frameB.covLeftRightMatches3D;

  // Reset the contents of the matched frames
  matchedFrameA = omsci::StereoFrame();
  matchedFrameB = omsci::StereoFrame();

  // For all frameA leftRightMatches
  while (frameALeftRightMatches.size() > 0) {
    // Grab and remove last element of frameA left to right matches
    cv::DMatch frameAMatch = frameALeftRightMatches.back();
    Eigen::Vector3d frameAMatch3D = frameALeftRightMatches3D.back();
    Eigen::Matrix3d frameACov3D = frameACovLeftRightMatches3D.back();
    frameALeftRightMatches.pop_back();
    frameALeftRightMatches3D.pop_back();
    frameACovLeftRightMatches3D.pop_back();

    // Get left and right descriptors in frameA
    cv::Mat descriptorAL, descriptorAR;
    frameA.leftDescriptors.row(frameAMatch.queryIdx).copyTo(descriptorAL);
    frameA.rightDescriptors.row(frameAMatch.trainIdx).copyTo(descriptorAR);

    // Loop through frameB left to right matches looking for matches to frameA
    // feature
    bool matchedAtoB = false;
    int j = 0;
    while (j < (int)frameBLeftRightMatches.size() && !matchedAtoB) {
      // Grab an element from frameB left to right matches
      cv::Mat descriptorBL, descriptorBR;
      cv::DMatch frameBMatch = frameBLeftRightMatches[j];
      Eigen::Vector3d frameBMatch3D = frameBLeftRightMatches3D[j];
      Eigen::Matrix3d frameBCov3D = frameBCovLeftRightMatches3D[j];

      frameB.leftDescriptors.row(frameBMatch.queryIdx).copyTo(descriptorBL);
      frameB.rightDescriptors.row(frameBMatch.trainIdx).copyTo(descriptorBR);

      // Attempt one-to-one matches from frame A left and right to frame B left
      // and right
      std::vector<cv::DMatch> matchAL2BL, matchAL2BR, matchAR2BL, matchAR2BR;
      matchFeatures(descriptorAL, descriptorBL, matchAL2BL);
      matchFeatures(descriptorAL, descriptorBR, matchAL2BR);
      matchFeatures(descriptorAR, descriptorBL, matchAR2BL);
      matchFeatures(descriptorAR, descriptorBR, matchAR2BR);

      // Check that there is a single four-way match between the left and right
      // images in frames A and B
      // TODO: decide between 4way or 2way match
      if (matchAL2BL.size() == 1 && matchAL2BR.size() == 1 &&
          matchAR2BL.size() == 1 && matchAR2BR.size() == 1) {
        // Match found
        matchedAtoB = true;

        // Add matched points
        matchesFrameA.push_back(frameAMatch);
        matchesFrameB.push_back(frameBMatch);
        points3DFrameA.push_back(frameAMatch3D);
        points3DFrameB.push_back(frameBMatch3D);
        covs3DFrameA.push_back(frameACov3D);
        covs3DFrameB.push_back(frameBCov3D);

        // Remove from frame B match candidates
        frameBLeftRightMatches.erase(frameBLeftRightMatches.begin() + j);
        frameBLeftRightMatches3D.erase(frameBLeftRightMatches3D.begin() + j);
        frameBCovLeftRightMatches3D.erase(frameBCovLeftRightMatches3D.begin() +
                                          j);
      }

      // Increment search index
      j++;
    }
  }

  // Create matched frames with their reduced set of matches (only including
  // frame to frame matches)
  matchedFrameA.leftDescriptors = frameA.leftDescriptors;
  matchedFrameA.rightDescriptors = frameA.rightDescriptors;
  matchedFrameA.leftKeypoints = frameA.leftKeypoints;
  matchedFrameA.rightKeypoints = frameA.rightKeypoints;
  matchedFrameA.leftRightMatches = matchesFrameA;
  matchedFrameA.leftRightMatches3D = points3DFrameA;
  matchedFrameA.covLeftRightMatches3D = covs3DFrameA;

  matchedFrameB.leftDescriptors = frameB.leftDescriptors;
  matchedFrameB.rightDescriptors = frameB.rightDescriptors;
  matchedFrameB.leftKeypoints = frameB.leftKeypoints;
  matchedFrameB.rightKeypoints = frameB.rightKeypoints;
  matchedFrameB.leftRightMatches = matchesFrameB;
  matchedFrameB.leftRightMatches3D = points3DFrameB;
  matchedFrameB.covLeftRightMatches3D = covs3DFrameB;
}

// Get the 3D vector from left camera to the centroid of all points in a
// StereoFrame
Eigen::Vector3d FeatureHandler::getCentroid(const omsci::StereoFrame& frame) {
  // Find the mean of all the points in the frame
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < frame.leftRightMatches3D.size(); i++) {
    mean += frame.leftRightMatches3D[i];
  }
  mean /= (double)frame.leftRightMatches3D.size();

  return mean;
}

// Get the minimum disparity
double FeatureHandler::getMinDisparity() { return minDisparity_; }

// Get the maximum disparity
double FeatureHandler::getMaxDisparity() { return maxDisparity_; }

}  // namespace sph
