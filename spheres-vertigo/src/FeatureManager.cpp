/**
 * @file FeatureManager.cpp
 * @brief Class handling the feature detection and matching.
 * @date November 02, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/FeatureManager.h"

namespace sph {

/* ************************************************************************** */
void FeatureManager::DetectAndCompute(StereoFrame *frame) const {
  SURFhandler_->detectAndCompute(frame->img_left, cv::noArray(),
                                 frame->info.keypts_left,
                                 frame->info.descr_left, false);
  SURFhandler_->detectAndCompute(frame->img_right, cv::noArray(),
                                 frame->info.keypts_right,
                                 frame->info.descr_right, false);

  // sort by v-coord to speed up search w/lookuptable (Tim Setterfield's idea)
  std::sort(frame->info.keypts_left.begin(), frame->info.keypts_left.end(),
            [](cv::KeyPoint a, cv::KeyPoint b) { return a.pt.y < b.pt.y; });
  std::sort(frame->info.keypts_right.begin(), frame->info.keypts_right.end(),
            [](cv::KeyPoint a, cv::KeyPoint b) { return a.pt.y < b.pt.y; });

  // mark frame as processed
  frame->processed = true;
}

/* ************************************************************************** */
void FeatureManager::DetectAndCompute(StereoFrame *frame,
                                      const cv::Mat &mask_left,
                                      const cv::Mat &mask_right) const {
  SURFhandler_->detectAndCompute(frame->img_left, mask_left,
                                 frame->info.keypts_left,
                                 frame->info.descr_left, false);
  SURFhandler_->detectAndCompute(frame->img_right, mask_right,
                                 frame->info.keypts_right,
                                 frame->info.descr_right, false);

  // sort by v-coord to speed up search w/lookuptable (Tim Setterfield's idea)
  std::sort(frame->info.keypts_left.begin(), frame->info.keypts_left.end(),
            [](cv::KeyPoint a, cv::KeyPoint b) { return a.pt.y < b.pt.y; });
  std::sort(frame->info.keypts_right.begin(), frame->info.keypts_right.end(),
            [](cv::KeyPoint a, cv::KeyPoint b) { return a.pt.y < b.pt.y; });

  // mark frame as processed
  frame->processed = true;
}

/* ************************************************************************** */
void FeatureManager::Match(StereoFrame *frame, const bool b_mask) const {
  std::vector<cv::DMatch> matches_all;

  if (b_mask) {
    // get mask and pass it to OpenCV matcher
    cv::Mat matchMask(frame->info.keypts_left.size(),
                      frame->info.keypts_right.size(), CV_8U, cv::Scalar(0));
    GetMatchMask(*frame, &matchMask);
    BFMatcher_->match(frame->info.descr_left, frame->info.descr_right,
                      matches_all, matchMask);
  } else {
    BFMatcher_->match(frame->info.descr_left, frame->info.descr_right,
                      matches_all, cv::noArray());
  }

  DiscardBadMatches(matches_all, &(frame->info.matches));
  // this->debug_stepThroughSingleFeatureMatches(frame);

  frame->matched = true;
}

/* ************************************************************************** */
void FeatureManager::Match(const cv::Mat &descr1, const cv::Mat &descr2,
                           std::vector<cv::DMatch> *matches) const {
  std::vector<cv::DMatch> matches_all;
  BFMatcher_->match(descr1, descr2, matches_all, cv::noArray());
  DiscardBadMatches(matches_all, matches);
}

/* ************************************************************************** */
void FeatureManager::Match(StereoFrame *A, StereoFrame *B) {
  StereoFrame matchedA = *A, matchedB = *B;
  matchedA.info.matches.clear();
  matchedA.info.matches3D.clear();
  matchedB.info.matches.clear();
  matchedB.info.matches3D.clear();
  std::vector<cv::DMatch> B_matches = B->info.matches;
  std::vector<Eigen::Vector3d> B_matches3D = B->info.matches3D;
  int A_idx = 0;

  for (auto matchA : A->info.matches) {
    cv::Mat descrAL, descrAR;
    A->info.descr_left.row(matchA.queryIdx).copyTo(descrAL);
    A->info.descr_right.row(matchA.trainIdx).copyTo(descrAR);
    float best_score = 1e6;
    int best_index = 0, current_B_index = 0;

    if (B_matches.size() > 0) {
      for (auto matchB : B_matches) {
        cv::Mat descrBL, descrBR;
        B->info.descr_left.row(matchB.queryIdx).copyTo(descrBL);
        B->info.descr_right.row(matchB.trainIdx).copyTo(descrBR);
        std::vector<cv::DMatch> matchAL2BL, matchAL2BR, matchAR2BL, matchAR2BR;

        // 4-way matching
        Match(descrAL, descrBL, &matchAL2BL);
        Match(descrAL, descrBR, &matchAL2BR);
        Match(descrAR, descrBL, &matchAR2BL);
        Match(descrAR, descrBR, &matchAR2BR);

        if (matchAL2BL.size() == 1 && matchAL2BR.size() == 1 &&
            matchAR2BL.size() == 1 &&
            matchAR2BR.size() == 1) {  // if it's 4way match, compare
          float score = matchAL2BL[0].distance + matchAL2BR[0].distance +
                        matchAR2BL[0].distance + matchAR2BR[0].distance;
          if (score < best_score) {
            best_score = score;
            best_index = current_B_index;
          }
        }
        current_B_index++;
      }
      if (best_score != 1e6) {
        matchedA.info.matches.emplace_back(matchA);  // Keep 4way match inliers
        matchedB.info.matches.emplace_back(B_matches[best_index]);

        matchedA.info.matches3D.emplace_back(A->info.matches3D[A_idx]);  // grab
        matchedB.info.matches3D.emplace_back(B_matches3D[best_index]);   // 3D

        B_matches.erase(B_matches.begin() + best_index);  // Discard used match
        B_matches3D.erase(B_matches3D.begin() + best_index);
      }
    } else {
      break;  // there are no more elements with which to match A's keypts
    }
    A_idx++;
  }
}

/* ************************************************************************** */
void FeatureManager::GetMatchMask(const StereoFrame &frame,
                                  cv::Mat *match_mask) const {
  // Create LUTs for masking (Tim Setterfield's idea)
  std::vector<int> lookuptable(frame.img_left.rows + 1);
  int kptIdx = 0;
  for (int row = 0; row < frame.img_left.rows + 1; row++) {
    // Checking for bounds with min and in while loop
    lookuptable[row] =
        std::min(static_cast<int>(frame.info.keypts_right.size()), kptIdx);
    while (kptIdx < static_cast<int>(frame.info.keypts_right.size()) &&
           std::floor(frame.info.keypts_right[kptIdx].pt.y) ==
               static_cast<float>(row)) {
      kptIdx++;
    }
  }
  // fill mask using vertical thresholds for fast, horizontal matching
  for (int i = 0; i < static_cast<int>(frame.info.descr_left.rows); i++) {
    int leftCoordV =
        static_cast<int>(std::round(frame.info.keypts_left.at(i).pt.y));
    int minRightCoordV = std::max(0, leftCoordV - max_vert_offset_);
    int maxRightCoordV =
        std::min(frame.img_left.rows + 1, leftCoordV + max_vert_offset_ + 1);
    for (int j = lookuptable[minRightCoordV]; j < lookuptable[maxRightCoordV];
         j++) {
      match_mask->at<uchar>(i, j) = 1;
    }
  }
}

/* ************************************************************************** */
void FeatureManager::DiscardBadMatches(
    const std::vector<cv::DMatch> &matches_all,
    std::vector<cv::DMatch> *matches) const {
  for (unsigned int i = 0; i < matches_all.size(); i++) {
    if (static_cast<double>(matches_all[i].distance) <= max_descr_distance_) {
      matches->emplace_back(matches_all[i]);
    }
  }
}

/* ************************************************************************** */
void FeatureManager::debug_stepThroughSingleFeatureMatches(
    StereoFrame *frame) const {
  // DEBUG
  std::vector<char> mask(frame->info.matches.size(), 0);
  for (unsigned int i = 0; i < frame->info.matches.size(); i++) {
    std::fill(mask.begin(), mask.end(), 0);
    mask.at(i) = 1;
    cv::drawMatches(frame->img_left, frame->info.keypts_left, frame->img_right,
                    frame->info.keypts_right, frame->info.matches,
                    frame->img_matches,
                    // CV_RGB(0,255,0), CV_RGB(255,0,0));
                    CV_RGB(0, 255, 0), CV_RGB(255, 0, 0), mask);
    std::cout
        << " i = " << i << std::endl
        << "  - keypt_left[queryIdx]  " << std::endl
        << "               : "
        << frame->info.keypts_left.at(frame->info.matches.at(i).queryIdx).pt
        << std::endl
        << "  - keypt_right[trainIdx] " << std::endl
        << "               : "
        << frame->info.keypts_right.at(frame->info.matches.at(i).trainIdx).pt
        << std::endl
        << "  - DMatch[" << i << "] : " << std::endl
        << "    - imgIdx   : " << frame->info.matches.at(i).imgIdx << std::endl
        << "    - queryIdx : " << frame->info.matches.at(i).queryIdx
        << std::endl
        << "    - trainIdx : " << frame->info.matches.at(i).trainIdx
        << std::endl;
    cv::imshow("left-right matches", frame->img_matches);
    cv::waitKey(0);
  }
}

/* ************************************************************************** */
void FeatureManager::MatchStereoFrames(const StereoFrame &frameA,
                                       const StereoFrame &frameB,
                                       StereoFrame *matchedFrameA,
                                       StereoFrame *matchedFrameB) {
  // Find all matches from frameA-to-frameB; each feature needs to be described
  // by four features.

  // Points, DMatches, and covariances from both frames, where the indices of
  // matching features are aligned.
  std::vector<Eigen::Vector3d> points3DFrameA, points3DFrameB;
  std::vector<cv::DMatch> matchesFrameA, matchesFrameB;
  std::vector<Eigen::Matrix3d> covs3DFrameA, covs3DFrameB;

  // Copy existing matches
  std::vector<cv::DMatch> frameALeftRightMatches = frameA.info.matches;
  std::vector<cv::DMatch> frameBLeftRightMatches = frameB.info.matches;
  std::vector<Eigen::Vector3d> frameALeftRightMatches3D = frameA.info.matches3D;
  std::vector<Eigen::Vector3d> frameBLeftRightMatches3D = frameB.info.matches3D;
  std::vector<Eigen::Matrix3d> frameACovLeftRightMatches3D =
      frameA.info.cov_matches3D;
  std::vector<Eigen::Matrix3d> frameBCovLeftRightMatches3D =
      frameB.info.cov_matches3D;

  // Reset the contents of the matched frames
  *matchedFrameA = frameA;
  *matchedFrameB = frameB;

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
    frameA.info.descr_left.row(frameAMatch.queryIdx).copyTo(descriptorAL);
    frameA.info.descr_right.row(frameAMatch.trainIdx).copyTo(descriptorAR);

    // Loop through frameB left to right matches looking for matches to frameA
    // feature
    bool matchedAtoB = false;
    int j = 0;
    while (j < static_cast<int>(frameBLeftRightMatches.size()) &&
           !matchedAtoB) {
      // Grab an element from frameB left to right matches
      cv::Mat descriptorBL, descriptorBR;
      cv::DMatch frameBMatch = frameBLeftRightMatches[j];
      Eigen::Vector3d frameBMatch3D = frameBLeftRightMatches3D[j];
      Eigen::Matrix3d frameBCov3D = frameBCovLeftRightMatches3D[j];

      frameB.info.descr_left.row(frameBMatch.queryIdx).copyTo(descriptorBL);
      frameB.info.descr_right.row(frameBMatch.trainIdx).copyTo(descriptorBR);

      // Attempt one-to-one matches from frame A left and right to frame B left
      // and right
      std::vector<cv::DMatch> matchAL2BL, matchAL2BR, matchAR2BL, matchAR2BR;
      Match(descriptorAL, descriptorBL, &matchAL2BL);
      Match(descriptorAL, descriptorBR, &matchAL2BR);
      Match(descriptorAR, descriptorBL, &matchAR2BL);
      Match(descriptorAR, descriptorBR, &matchAR2BR);

      // Check that there is a single four-way match between the left and right
      // images in frames A and B
      // TODO(tim): decide between 4way or 2way match
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
  matchedFrameA->info.descr_left = frameA.info.descr_left;
  matchedFrameA->info.descr_right = frameA.info.descr_right;
  matchedFrameA->info.keypts_left = frameA.info.keypts_left;
  matchedFrameA->info.keypts_right = frameA.info.keypts_right;
  matchedFrameA->info.matches = matchesFrameA;
  matchedFrameA->info.matches3D = points3DFrameA;
  matchedFrameA->info.cov_matches3D = covs3DFrameA;

  matchedFrameB->info.descr_left = frameB.info.descr_left;
  matchedFrameB->info.descr_right = frameB.info.descr_right;
  matchedFrameB->info.keypts_left = frameB.info.keypts_left;
  matchedFrameB->info.keypts_right = frameB.info.keypts_right;
  matchedFrameB->info.matches = matchesFrameB;
  matchedFrameB->info.matches3D = points3DFrameB;
  matchedFrameB->info.cov_matches3D = covs3DFrameB;
}

}  // namespace sph
