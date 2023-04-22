/**
 * @file FeatureManager.h
 * @brief Class handling the feature detection and matching.
 * @date November 02, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_FEATUREMANAGER_H_
#define SPH_VERTIGO_FEATUREMANAGER_H_

#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @brief Class for processing left and right stereo images using SURF.
 */
class FeatureManager {
 public:
  /**
   * @brief Default constructor uses SURF features with brute force matcher.
   */
  FeatureManager() {}

  /**
   * @brief Default destructor.
   */
  ~FeatureManager() {}

  /**
   * @brief Fills `StereoFrame`'s `StereoInfo` using left and right images.
   * @param[in/out] frame Loaded VERTIGO stereo pair frame.
   *
   * Wrapper for OpenCV's `detectAndCompute` function. Keypoints and descriptors
   * are stored in `StereoInfo`, with the left image's keypoints sorted by
   * y-coordinate for speeding up correspondence search in `match`.
   */
  void DetectAndCompute(StereoFrame *frame) const;

  /**
   * @brief Overload to supports custom input mask for left and right images.
   * @param[in/out] frame Loaded VERTIGO stereo pair frame.
   * @param[in] mask_left Container denoting are in which to perform detection.
   * @param[in] mask_right Container denoting are in which to perform detection.
   */
  void DetectAndCompute(StereoFrame *frame, const cv::Mat &mask_left,
                        const cv::Mat &mask_right) const;

  /**
   * @brief Match keypoints from `StereoFrames`'s left and right image.
   * @param[in/out] frame Loaded VERTIGO stereo pair frame.
   * @param[in] b_mask Compute and use boolean mask to speed up matching?
   *
   * Uses computed descriptors to match left and right keypoints, and store the
   * correspondences. By default, keypoints outside the keypoint's pixel space
   * threshold are ignored through the creation of a lookup table and mask.
   */
  void Match(StereoFrame *frame, const bool b_mask = true) const;

  /**
   * @brief Overloaded function without the `StereoFrame` structure
   * @param[in] descr1 First set of image descriptors.
   * @param[in] descr2 Second set of image descriptors.
   * @param[out] matches Vector containing the matches between the two sets.
   */
  void Match(const cv::Mat &descr1, const cv::Mat &descr2,
             std::vector<cv::DMatch> *matches) const;

  /**
   * @brief Overload to perform 4-way matching between two `StereoFrames`.
   * @param[in/out] A Processed stereo pair used for feature matching.
   * @param[in/out] B Processed stereo pair used for feature matching.
   *
   * Results are stored within both `StereoFrame` objects.
   */
  void Match(StereoFrame *A, StereoFrame *B);

  /**
   * @brief Create Lookup table using sorted v-coord kpts to make match mask.
   * @param[in] frame Frame from which to lookup the keypoints.
   * @param[out] match_mask Boolean mask that eliminates inconsistent matches.
   */
  void GetMatchMask(const StereoFrame &frame, cv::Mat *match_mask) const;

  /**
   * @brief Discards matches that don't meet the minimum criteria requirement.
   * @param[in] matches_all Vector with initial keypoint matches.
   * @param[out] matches Vector with filtered keypoint matches.
   */
  void DiscardBadMatches(const std::vector<cv::DMatch> &matches_all,
                         std::vector<cv::DMatch> *matches) const;

  /**
   * @brief Match two stereo frames.
   * @param[in] frameA Stereo pair to be matched.
   * @param[in] frameB Stereo pair to be matched.
   * @param[out] matchedFrameA Frame struct with only matches information.
   * @param[out] matchedFrameB Frame struct with only matches information.
   *
   * Match two stereo frames, returning stereo frames containing only the subset
   * of leftRightMatches(3D) that also matches frame to frame.
   */
  void MatchStereoFrames(const StereoFrame &frameA, const StereoFrame &frameB,
                         StereoFrame *matchedFrameA,
                         StereoFrame *matchedFrameB);

 private:
  // Tuning parameters
  double max_descr_distance_{0.9};  //!< Threshold for real-valued descriptors.
  int max_vert_offset_{2};  ///< Param for stereo correspondence matching [px].
  double hessian_thresh_{400};  // 400 target, 2400 room (ISS scenes).

  /// All in one SURF feature handler
  cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> SURFhandler_ =
      cv::xfeatures2d::SURF::create(hessian_thresh_,
                                    4,       // nOctaves
                                    4,       // nOctaveLayers
                                    true,    // extended
                                    false);  // upright

  //! Brute force matcher with masking support.
  cv::Ptr<cv::BFMatcher> BFMatcher_ = cv::BFMatcher::create();

 private:
  void debug_stepThroughSingleFeatureMatches(StereoFrame *frame) const;
};

}  // namespace sph

#endif  // SPH_VERTIGO_FEATUREMANAGER_H_
