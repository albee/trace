#ifndef _FEATUREHANDLER_H_
#define _FEATUREHANDLER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/xfeatures2d/nonfree.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

class FeatureHandler {
 public:
  // Construction / Destruction
  FeatureHandler(int* detectorExtractorMatcherTypes,
                 cv::FeatureDetector* featureDetector,
                 cv::DescriptorExtractor* descriptorExtractor,
                 cv::DescriptorMatcher* descriptorMatcher,
                 sph::StereoCamera* camera);
  ~FeatureHandler();

  // General
  void getFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                   cv::Mat& descriptors, const cv::Mat& mask = cv::Mat());
  void getRowLookupTable(const std::vector<cv::KeyPoint>& keypoints,
                         int imageHeight, std::vector<int>& lookupTable);
  void matchFeatures(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                     std::vector<cv::DMatch>& matches,
                     const cv::Mat& mask = cv::Mat());
  void getAverageTimes(double& detectTime, double& extractTime,
                       double& matchTime, double& triangulateTime);

  // Stereo
  void setMinMaxDepth(double min, double max);
  void setMaxDescriptorDistance(double maxDist);
  void setCustomStereoFeatureMask(const cv::Mat& leftFeatureMask,
                                  const cv::Mat& rightFeatureMask);
  void getStereoFrame(const cv::Mat& leftImage, const cv::Mat& rightImage,
                      omsci::StereoFrame& frame);
  void showStereoFeatures(const cv::Mat& leftImage, const cv::Mat& rightImage,
                          const omsci::StereoFrame& frame, cv::Mat& outImage);
  void showStereoFeatures(const cv::Mat& leftImage, const cv::Mat& rightImage,
                          const omsci::StereoFrame& frame, cv::Mat& outImage,
                          cv::Scalar rgbLR);
  void showStereoFrame(const cv::Mat& leftImage, const cv::Mat& rightImage,
                       const omsci::StereoFrame& frame, cv::Mat& outImage);
  void showStereoFrame(const cv::Mat& leftImage, const cv::Mat& rightImage,
                       const omsci::StereoFrame& frame, cv::Mat& outImage,
                       cv::Scalar rgbLR);
  void showMatchedStereoFrames(const cv::Mat& leftImageB,
                               const cv::Mat& rightImageB,
                               const omsci::StereoFrame& matchedFrameA,
                               const omsci::StereoFrame& matchedFrameB,
                               cv::Mat& outImage);
  void showMatchedStereoFrames(const cv::Mat& leftImageB,
                               const cv::Mat& rightImageB,
                               const omsci::StereoFrame& matchedFrameA,
                               const omsci::StereoFrame& matchedFrameB,
                               cv::Mat& outImage, cv::Scalar rgbLR,
                               cv::Scalar rgbAB);
  void mergeStereoFrames(omsci::StereoFrame& frameA, omsci::StereoFrame& frameB,
                         omsci::StereoFrame& frameAB);
  void triangulate(omsci::StereoFrame& frame);
  Eigen::Vector2d reproject(const Eigen::Vector3d& point3D);
  void matchStereoFrames(const omsci::StereoFrame& frameA,
                         const omsci::StereoFrame& frameB,
                         omsci::StereoFrame& matchedFrameA,
                         omsci::StereoFrame& matchedFrameB);
  Eigen::Vector3d getCentroid(const omsci::StereoFrame& frame);
  double getMinDisparity();
  double getMaxDisparity();

 private:
  // Variables -----------------------------------------

  // Pointers to feature detector, descriptor extractor, and descriptor matcher
  cv::FeatureDetector* featureDetector_;
  cv::DescriptorExtractor* descriptorExtractor_;
  cv::DescriptorMatcher* descriptorMatcher_;

  // Feature detector type from omsci::FeatureDetectorType enum
  // Types of detector:	SIFT, SURF, ORB, BRISK, FAST, GFTT, STAR, MSER, GFTT,
  // HARRIS, Dense, SimpleBlob (OpenCV 2.4.6.1)
  int featureDetectorType_;

  // Descriptor extractor type from omsci::DescriptorExtractorType enum
  // Types of descriptor: SIFT, SURF, ORB, BRISK, BRIEF, FREAK (OpenCV 2.4.6.1)
  int descriptorExtractorType_;

  // Integer indicating descriptor matcher type from
  // omsci::DescriptorMatcherType enum Type of matcher: BruteForce,
  // BruteForce-L1, BruteForce-Hamming, BruteForce-Hamming(2), FlannBased
  // (OpenCV 2.4.6.1)
  int descriptorMatcherType_;

  // Whether to sort features by v coordinate
  bool sortingOn_;

  // The maximum 'distance' between descriptors to consider it a match
  double maxDescriptorDistance_;

  // Maximum difference between left and right vertical coords [px]
  int triangulatorMaxVerticalOffset_;

  // Minimum and maximum depths of features to keep for stereo images [m]
  double minDepth_, maxDepth_;

  // Minimum and maximum disparity based on maximum depth [px]
  double minDisparity_, maxDisparity_;

  // Image masks for faster feature detection in left and right images
  cv::Mat leftFeatureMask_, rightFeatureMask_;

  // Boolean indicating whether or not to use custom feature mask
  bool useCustomFeatureMask_;

  // Total amount of time spent detecting, extracting, matching, and
  // triangulating features [ms]
  double totalTimeDetectFunc_, totalTimeExtractFunc_, totalTimeMatchFunc_,
      totalTimeTrianFunc_;

  // Total number of times detecting features, extracting features, and matching
  // features [-]
  unsigned int totalNumDetectFunc_, totalNumExtractFunc_, totalNumMatchFunc_,
      totalNumTrianFunc_;

  // Camera structure pointer if stereo camera being used
  StereoCamera* stereoCamera_;

  /// All in one SURF feature handler
  cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> SURFhandler_ =
      cv::xfeatures2d::SURF::create(400,
                                    4,       // nOctaves
                                    4,       // nOctaveLayers
                                    true,    // extended
                                    false);  // upright

  //! Brute force matcher with masking support.
  cv::Ptr<cv::BFMatcher> BFMatcher_ = cv::BFMatcher::create();
};

}  // namespace sph

#endif /* _FEATUREHANDLER_H_ */
