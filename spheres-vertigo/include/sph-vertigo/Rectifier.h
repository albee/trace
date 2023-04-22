/**
 * @file Rectifier.h
 * @brief Undistortion of images through rectification maps.
 * @date December 5, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_RECTIFIER_H_
#define SPH_VERTIGO_RECTIFIER_H_

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

//! Class for rectification purposes and handling camera calibration parameters
/*!
  Reads in extrinsic and intrinsic calibration parameters from OpenCV-style
  yaml files, and allows for recitifaction map calculation.
 */
class Rectifier {
 private:
  //! Stores string to directory in which the calibration files are stored
  std::string calib_dir_;

  /* extrinsic parameters */
  cv::Mat R_,  //! Rotation matrix from left to right camera
      T_;      //! Translation vector from left to right camera focal points.

  /* Rectified parameters */
  cv::Mat R1_,  //! Rectification rotation matrix to make image planes the same
      R2_,      //! Rectification rotation matrix to make image planes the same
      P1_,      //! 3x4 projection matrix in new (rectified) coordinate system
      P2_,      //! 3x4 projection matrix in new (rectified) coordinate system
      Q_,       //! 4x4 disparity-to-depth mapping matrix (reprojection matrix)
      F_;

  /* intrinsic parameters */
  cv::Mat M1_,  //! Camera matrix for left camera
      D1_,      //! Distortion coefficients for left camera
      M2_,      //! Camera matrix for right camera
      D2_;      //! Distortion coefficients for right camera

  /* triangulation parameters */
  double f_,   //! Rectified stereo rig focal distance
      cx_,     //! Rectified stereo rig principal point x-coordinate
      cy_;     //! Rectified stereo rig principal point y-coordinate
  double Tx_,  //! Stereo baseline
      Ty_, Tz_;

  /* rectification maps */
  cv::Mat left_rect_map1_, left_rect_map2_, right_rect_map1_, right_rect_map2_;

  /* valid regoins of interest post rectification */
  cv::Rect roi1_, roi2_;

 public:
  /* constructors */
  Rectifier(std::string calib_dir);

  /* destructors */
  virtual ~Rectifier();

  /* calculate rectification maps for both cameras */
  int calcRectificationMaps();

  /* rectify images using calculated maps */
  int rectifyStereoFrame(cv::Mat &leftImage, cv::Mat &rightImage);

  /* assign calibration parameters to stereo cam struct */
  void getTriangulationParameters(StereoCamera &cam);

  /* create cam with triangulation params */
  std::shared_ptr<StereoCamera> createStereoCam();
  void getQ(cv::Mat &Q) const { Q = Q_; };

  /* operator overloading */
  friend std::ostream &operator<<(std::ostream &os, const Rectifier &r);

}; /* Rectifiers*/

}  // namespace sph

#endif  // SPH_VERTIGO_RECTIFIER_H_
