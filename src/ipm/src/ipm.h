#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct MEICamera {
  MEICamera(const std::string &intrinsicFile, const std::string &extrinsicFile);
  ~MEICamera() = default;

  void DebugString() const;

  bool is_valid_{false};
  std::string id_;
  int height_, width_;
  double xi_;
  cv::Mat K_, D_;
  Eigen::Affine3d T_vehicle_cam_; // Extrinsic: from camera to vehicle
  Eigen::Affine3d T_cam_vehicle_; // Extrinsic: from camera to vehicle
};

class IPM {
public:
  IPM();
  ~IPM() = default;

  void AddCamera(const std::string &intrinsicsFile,
                 const std::string &extrinsicFile);

  cv::Mat GenerateIPMImage(const std::vector<cv::Mat> &images) const;

private:
  std::vector<MEICamera> cameras_;

  int ipm_img_h_ = 1000;
  int ipm_img_w_ = 1000;
  const double pixel_scale_ = 0.02;
  const double ipm_img_height_ = 0.0;
};