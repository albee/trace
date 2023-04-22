/**
 * @file Rectifier.cpp
 * @brief Undistortion of images through rectification maps.
 * @date December 5, 2018
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran <teran@mit.edu>
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/Rectifier.h"

namespace sph {

/* ************************************************************************* */
Rectifier::Rectifier(std::string calib_dir) : calib_dir_(calib_dir) {
  std::cout << "Created Rectifier object. Looking for calibration \n"
            << "parameters in " << calib_dir_ << std::endl;

  std::string extrinsic_file = calib_dir_ + "/extrinsics.yml",
              intrinsic_file = calib_dir_ + "/intrinsics.yml";

  cv::FileStorage fs(extrinsic_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "no extrinsic_file found! exiting..." << std::endl;
  } else {
    std::cout << "extracting extrinsic values.." << std::endl;
    fs["R"] >> R_;
    fs["T"] >> T_;
    fs["F"] >> F_;
  }

  fs.open(intrinsic_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "no intrinsic_file found! exiting..." << std::endl;
  } else {
    std::cout << "extracting intrinsic values..." << std::endl;
    fs["M1"] >> M1_;
    fs["M2"] >> M2_;
    fs["D1"] >> D1_;
    fs["D2"] >> D2_;
  }
}

/* ************************************************************************* */
Rectifier::~Rectifier() {}

/* ************************************************************************* */
int Rectifier::calcRectificationMaps() {
  /* compute rectification transforms for each camera */
  cv::stereoRectify(M1_, D1_, M2_, D2_, cv::Size(640, 480), R_, T_, R1_, R2_,
                    P1_, P2_, Q_, cv::CALIB_ZERO_DISPARITY, 0,
                    cv::Size(640, 480), &roi1_, &roi2_);

  /* calculate the rectification maps */
  cv::initUndistortRectifyMap(M1_, D1_, R1_, P1_, cv::Size(640, 480), CV_16SC2,
                              left_rect_map1_, left_rect_map2_);
  cv::initUndistortRectifyMap(M2_, D2_, R2_, P2_, cv::Size(640, 480), CV_16SC2,
                              right_rect_map1_, right_rect_map2_);

  /* fetch triangulation parameters */
  f_ = P1_.at<double>(0, 0);
  cx_ = P2_.at<double>(0, 2);
  cy_ = P2_.at<double>(1, 2);
  Tx_ =
      T_.at<double>(0, 0) * 0.0254 / (6 * 1.0e-6);  // get translation in pixels
  Ty_ = T_.at<double>(0, 1) * 0.0254 / (6 * 1.0e-6);  // chckbrd sqr sz: 2.54cm
  Tz_ = T_.at<double>(0, 2) * 0.0254 / (6 * 1.0e-6);  // pxl sz: 6um

  std::cout << "maps and triangulation params obtained" << std::endl;
  return (0);
}

/* ************************************************************************* */
int Rectifier::rectifyStereoFrame(cv::Mat &leftImage, cv::Mat &rightImage) {
  cv::Mat tmpLeft, tmpRight;
  cv::remap(leftImage, tmpLeft, left_rect_map1_, left_rect_map2_,
            cv::INTER_LINEAR);
  cv::remap(rightImage, tmpRight, right_rect_map1_, right_rect_map2_,
            cv::INTER_LINEAR);

  /* copy images to input container */
  leftImage = tmpLeft;
  rightImage = tmpRight;
  return (0);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Rectifier &r) {
  os << " * Rectifier with the following calibration parameters: " << std::endl
     << "  - f  : " << r.f_ << " px" << std::endl
     << "  - cx : " << r.cx_ << " px" << std::endl
     << "  - cy : " << r.cy_ << " px" << std::endl
     << "  - Tx : " << r.Tx_ << " px" << std::endl
     << "  - Ty : " << r.Ty_ << " px" << std::endl
     << "  - Tz : " << r.Tz_ << " px" << std::endl
     << " * P1 values: " << std::endl
     << "  - f  : " << r.P1_.at<double>(0, 0) << std::endl
     << "  - cx : " << r.P1_.at<double>(0, 2) << std::endl
     << "  - cy : " << r.P1_.at<double>(1, 2) << std::endl
     << " * P2 values: " << std::endl
     << "  - f  : " << r.P2_.at<double>(0, 0) << std::endl
     << "  - cx : " << r.P2_.at<double>(0, 2) << std::endl
     << "  - cy : " << r.P2_.at<double>(1, 2) << std::endl
     << " * M1 values: " << std::endl
     << "  - f  : " << r.M1_.at<double>(0, 0) << std::endl
     << "  - cx : " << r.M1_.at<double>(0, 2) << std::endl
     << "  - cy : " << r.M1_.at<double>(1, 2) << std::endl
     << " * M2 values: " << std::endl
     << "  - f  : " << r.M2_.at<double>(0, 0) << std::endl
     << "  - cx : " << r.M2_.at<double>(0, 2) << std::endl
     << "  - cy : " << r.M2_.at<double>(1, 2) << std::endl;
  return os;
}

/* ************************************************************************* */
std::shared_ptr<StereoCamera> Rectifier::createStereoCam() {
  auto cam = std::make_shared<StereoCamera>();
  cam->f = f_;
  cam->cx = cx_;
  cam->cy = cy_;                            // [px]
  cam->tx = -T_.at<double>(0, 0) * 0.0254;  // [m], chkbrd sqr sz: 2.54cm
  cam->width = 640;
  cam->height = 480;          // NOTE hardcoded for VERTIGO
  cam->covS << 0.6944, 0, 0,  // NOTE hardcoded for VERTIGO
      0, 0.6944, 0,           // from Tim Setterfield's notes
      0, 0, 0.6944;           // 2.5px = 3-sigma, sigma^2=( 2.5/3 )^2
  return (cam);
}

}  // namespace sph
