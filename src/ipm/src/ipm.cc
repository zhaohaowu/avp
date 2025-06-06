#include "ipm.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

MEICamera::MEICamera(const std::string& intrinsicFile,
                     const std::string& extrinsicFile) {
  K_ = cv::Mat::eye(3, 3, CV_64F);
  D_ = cv::Mat::zeros(4, 1, CV_64F);

  {  // load intrinsic
    cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open intrinsic file: " << intrinsicFile
                << std::endl;
      return;
    }
    cv::FileNode m_param = fs["mirror_parameters"];
    cv::FileNode D_param = fs["distortion_parameters"];
    cv::FileNode K_param = fs["projection_parameters"];
    if (m_param.empty() || D_param.empty() || K_param.empty()) {
      std::cerr << "Error intrinsic file: " << intrinsicFile << std::endl;
      return;
    }
    fs["image_width"] >> width_;
    fs["image_height"] >> height_;
    m_param["xi"] >> xi_;
    K_param["gamma1"] >> K_.at<double>(0, 0);
    K_param["gamma2"] >> K_.at<double>(1, 1);
    K_param["u0"] >> K_.at<double>(0, 2);
    K_param["v0"] >> K_.at<double>(1, 2);
    D_param["k1"] >> D_.at<double>(0, 0);
    D_param["k2"] >> D_.at<double>(1, 0);
    D_param["p1"] >> D_.at<double>(2, 0);
    D_param["p2"] >> D_.at<double>(3, 0);
  }

  {  // load extrinsic
    cv::FileStorage fs(extrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open extrinsic file: " << extrinsicFile
                << std::endl;
      return;
    }
    cv::FileNode trans_param = fs["transform"]["translation"];
    cv::FileNode rot_param = fs["transform"]["rotation"];
    if (trans_param.empty() || rot_param.empty()) {
      std::cerr << "Error extrinsic file: " << extrinsicFile << std::endl;
      return;
    }
    fs["frame_id"] >> id_;

    // T_vehicle_cam_ represents the transformation from the camera coordinate system to the vehicle coordinate system,
    // the origin of the vehicle coordinate system is the center of the vehicle.
    trans_param["x"] >> T_vehicle_cam_.translation().x();
    trans_param["y"] >> T_vehicle_cam_.translation().y();
    trans_param["z"] >> T_vehicle_cam_.translation().z();
    Eigen::Quaterniond q;
    rot_param["x"] >> q.x();
    rot_param["y"] >> q.y();
    rot_param["z"] >> q.z();
    rot_param["w"] >> q.w();
    T_vehicle_cam_.linear() = q.toRotationMatrix();
    T_cam_vehicle_ = T_vehicle_cam_.inverse();
  }

  is_valid_ = true;
}

void MEICamera::DebugString() const {
  std::cout << id_ << " : " << is_valid_ << std::endl;
  std::cout << "height: " << height_ << std::endl;
  std::cout << "width: " << width_ << std::endl;
  std::cout << "xi: " << xi_ << std::endl;
  std::cout << "K : " << K_ << std::endl;
  std::cout << "D : " << D_ << std::endl;
  std::cout << "cam_extrinsic : " << T_vehicle_cam_.matrix() << std::endl;
}

IPM::IPM() {}

void IPM::AddCamera(const std::string& intrinsicsFile,
                    const std::string& extrinsicFile) {
  cameras_.emplace_back(intrinsicsFile, extrinsicFile);
  std::cout << "---------ipm add new camera ---------" << std::endl;
  cameras_.back().DebugString();
}

cv::Mat IPM::GenerateIPMImage(const std::vector<cv::Mat>& images) const {
  // Initialize a black IPM image with dimensions ipm_img_h_ x ipm_img_w_ and 3 channels (RGB)
  cv::Mat ipm_image = cv::Mat::zeros(ipm_img_h_, ipm_img_w_, CV_8UC3);

  // Check if the number of input images matches the number of cameras
  if (images.size() != cameras_.size()) {
    // If not, print an error message and return the initialized black IPM image
    std::cout << "IPM not init normaly !" << std::endl;
    return ipm_image;
  }
  // Iterate over each pixel in the IPM image
  for (int u = 0; u < ipm_img_w_; ++u) {
    for (int v = 0; v < ipm_img_h_; ++v) {
      // Calculate the point p_v in vehicle coordinates, p_v is corresponding to the current pixel (u, v).
      // Assume the height of the ipm_image in vehicle coordinate is 0.
      Eigen::Vector3d p_v(-(0.5 * ipm_img_h_ - u) * pixel_scale_,
                          (0.5 * ipm_img_w_ - v) * pixel_scale_, 0);

      // Iterate over each camera
      for (size_t i = 0; i < cameras_.size(); ++i) {
        // Project the vehile point p_v into the image plane uv
        ////////////////////////TODO begin///////////////////////
        int uv0;
        int uv1;
        Eigen::Vector3d p_c = cameras_[i].T_cam_vehicle_ * p_v;
        Eigen::Vector3d p_c_norm = p_c / p_c.norm();
        Eigen::Vector3d p_cp = p_c_norm;
        if (p_cp[2] < 0) {
          // std::cout << "过滤相机后面的点" <<std::endl;
          continue;
        }
        p_cp[2] += cameras_[i].xi_;
        Eigen::Vector2d p_cp_norm = {p_cp[0] / p_cp[2], p_cp[1] / p_cp[2]};
        uv0 = cameras_[i].K_.at<double>(0, 0) * p_cp_norm[0] + cameras_[i].K_.at<double>(0, 2);
        uv1 = cameras_[i].K_.at<double>(1, 1) * p_cp_norm[1] + cameras_[i].K_.at<double>(1, 2);
        ////////////////////////TODO end/////////////////////
        // (uv0, uv1) is the projected pixel from p_v to cameras_[i]
        // Skip this point if the projected coordinates are out of bounds of the camera image
        if (uv0 < 0 || uv0 >= cameras_[i].width_ || uv1 < 0 ||
            uv1 >= cameras_[i].height_) {
          continue;
        }

        // Get the pixel color from the camera image and set it to the IPM image
        // If the IPM image pixel is still black (not yet filled), directly assign the color
        if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
          ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
        } else {
          // Otherwise, average the existing color with the new color
          // ipm_image.at<cv::Vec3b>(v, u) = (ipm_image.at<cv::Vec3b>(v, u) +
          //                                  images[i].at<cv::Vec3b>(uv1, uv0)) /
          //                                 2;
        }
      }
    }
  }

  // Return the generated IPM image
  return ipm_image;
}