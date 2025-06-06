#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "map.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/PointCloud.h"
#include "slot.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

struct TimedPose {
  double time_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond R_;
};
class RosViewer {
public:
  RosViewer(ros::NodeHandle &n) : nh_(n) {
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_global_dash_pts_ =
        n.advertise<sensor_msgs::PointCloud>("global_dash_pts", 1000);
    pub_global_arrow_pts_ =
        n.advertise<sensor_msgs::PointCloud>("global_arrow_pts", 1000);
    pub_global_slot_pts_ =
        n.advertise<sensor_msgs::PointCloud>("global_slot_pts", 1000);
    pub_mesh_ = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    pub_slot_marker_ =
        n.advertise<visualization_msgs::Marker>("slot_vector", 100, true);
    pub_slot_marker_ipm_ =
        n.advertise<visualization_msgs::Marker>("slot_vector_ipm", 100, true);
    path_.header.frame_id = "world";
  }
  ~RosViewer() { ros::shutdown(); }

  geometry_msgs::Point to_ros(const Eigen::Vector3d &pt) {
    geometry_msgs::Point ret;
    ret.x = pt.x();
    ret.y = pt.y();
    ret.z = pt.z();
    return ret;
  }

  cv::Mat drawIPMImg(const std::vector<cv::Point2f> &corners,
                     const std::vector<cv::Point2f> &slots_corners,
                     const cv::Mat &ipm_image) {
    auto ret = ipm_image.clone();
    for (const auto &center : corners) {
      cv::circle(ret, center, 50, 255, 2);
    }
    for (int i = 0; i < slots_corners.size(); i += 4) {
      cv::line(ret, slots_corners[i], slots_corners[i + 1], 0, 2);
      cv::line(ret, slots_corners[i + 2], slots_corners[i + 1], 0, 2);
      cv::line(ret, slots_corners[i + 2], slots_corners[i + 3], 0, 2);
      cv::line(ret, slots_corners[i], slots_corners[i + 3], 0, 2);
    }
    return ret;
  }

  void displayIpmDetection(const std::vector<cv::Point2f> &corners,
                           const std::vector<cv::Point2f> &slots_corners,
                           const cv::Mat &ipm_image) {
    std::cout << "\tipm detect : " << corners.size() << " corners, "
              << slots_corners.size() / 4 << " slots" << std::endl;

    cv::imshow("ipm_detection_result",
               drawIPMImg(corners, slots_corners, ipm_image));
  }

  void displayAvpMap(const TimedPose &pose, const Map &avp_map_,
                     const std::vector<Slot> &cur_slots) {
    // publish pose
    publishPose(pose);
    // publish slot
    publishSlots(avp_map_.getAllSlots(), pose.time_, pub_slot_marker_, true);
    publishSlots(cur_slots, pose.time_, pub_slot_marker_ipm_, false);
    // publish semantic points
    auto grid_arrow = avp_map_.getSemanticElement(SemanticLabel::kArrowLine);
    publishPoints({grid_arrow.begin(), grid_arrow.end()}, pose.time_,
                  pub_global_arrow_pts_);
    auto grid_dash = avp_map_.getSemanticElement(SemanticLabel::kDashLine);
    publishPoints({grid_dash.begin(), grid_dash.end()}, pose.time_,
                  pub_global_dash_pts_);

    ros::spinOnce();
  }

private:
  void publishSlots(const std::vector<Slot> &slots, double time,
                    ros::Publisher &publisher, bool avp_slots) {
    if (slots.empty()) {
      return;
    }
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time(time);
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.id = 0;

    if (avp_slots) {
      line_list.color.g = 1.0;
    } else {
      line_list.color.r = 1.0;
    }
    line_list.color.a = 1.0;

    for (const auto &slot : slots) {
      auto p0 = to_ros(slot.corners_[0]);
      auto p1 = to_ros(slot.corners_[1]);
      auto p2 = to_ros(slot.corners_[2]);
      auto p3 = to_ros(slot.corners_[3]);
      line_list.points.push_back(p0);
      line_list.points.push_back(p1);
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
      line_list.points.push_back(p2);
      line_list.points.push_back(p3);
      line_list.points.push_back(p3);
      line_list.points.push_back(p0);
    }
    publisher.publish(line_list);
  }

  void publishPoints(const std::vector<Eigen::Vector3i> &grid, double time,
                     ros::Publisher &publisher) {
    sensor_msgs::PointCloud global_cloud;
    global_cloud.header.frame_id = "world";
    global_cloud.header.stamp = ros::Time(time);
    for (Eigen::Vector3i pt : grid) {
      geometry_msgs::Point32 p;
      p.x = pt.x() * kPixelScale;
      p.y = pt.y() * kPixelScale;
      p.z = pt.z() * kPixelScale;
      global_cloud.points.push_back(p);
    }
    publisher.publish(global_cloud);
  }

  void publishPose(const TimedPose &pose) {
    Eigen::Vector3d position = pose.t_;
    Eigen::Quaterniond q = Eigen::Quaterniond(pose.R_);

    // pub odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(pose.time_);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_odometry_.publish(odometry);
    // pub path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(pose.time_);
    pose_stamped.pose = odometry.pose.pose;
    path_.poses.push_back(pose_stamped);
    if (path_.poses.size() > 10000) {
      path_.poses.erase(path_.poses.begin());
    }
    pub_path_.publish(path_);
    // pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    transform.setOrigin(tf::Vector3(position(0), position(1), position(2)));
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(pose.time_),
                                          "world", "vehicle"));
    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(pose.time_);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d frontleftup2rightfrontup;
    frontleftup2rightfrontup << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * frontleftup2rightfrontup.transpose() * rot_mesh;
    Eigen::Vector3d t_mesh = q * Eigen::Vector3d(0, 1.5, 0) + position;
    meshROS.pose.orientation.w = q_mesh.w();
    meshROS.pose.orientation.x = q_mesh.x();
    meshROS.pose.orientation.y = q_mesh.y();
    meshROS.pose.orientation.z = q_mesh.z();
    meshROS.pose.position.x = t_mesh(0);
    meshROS.pose.position.y = t_mesh(1);
    meshROS.pose.position.z = t_mesh(2);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    std::string mesh_resource("package://avp_mapping/meshes/car.dae");
    meshROS.mesh_resource = mesh_resource;
    pub_mesh_.publish(meshROS);
  }

  ros::NodeHandle &nh_;
  nav_msgs::Path path_; // trajectory
  ros::Publisher pub_path_, pub_odometry_, pub_mesh_;
  ros::Publisher pub_global_dash_pts_, pub_global_arrow_pts_,
      pub_global_slot_pts_;
  ros::Publisher pub_slot_marker_, pub_slot_marker_ipm_;
};

class Pipeline {
private:
  ros::NodeHandle nh_;
  std::deque<TimedPose> pose_queue_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_img_;
  Map avp_map_; // avp map data
  RosViewer &viewer_;

public:
  Pipeline(ros::NodeHandle &n, RosViewer &viewer) : nh_(n), viewer_(viewer) {
    sub_pose_ = nh_.subscribe<nav_msgs::Odometry>(
        "/gps_odom", 10, &Pipeline::PoseCallback, this);
    sub_img_ = nh_.subscribe<sensor_msgs::CompressedImage>(
        "/ipm_label", 10, &Pipeline::ImageCallback, this);
  }

  void PoseCallback(const nav_msgs::OdometryConstPtr &msg) {
    Eigen::Affine3d T_w_v = Eigen::Affine3d::Identity();
    auto q = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q.normalize();
    T_w_v.linear() = q.toRotationMatrix();
    T_w_v.translation() =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    static Eigen::Affine3d T_first = T_w_v;
    // T_w_v = T_first.inverse() * T_w_v;
    TimedPose pose;
    pose.time_ = msg->header.stamp.toSec();
    pose.R_ = T_w_v.linear();
    pose.t_ = T_w_v.translation();
    pose_queue_.emplace_back(pose);
    if (pose_queue_.size() > 100) {
      pose_queue_.pop_front();
    }
    // static std::vector<TimedPose> poses;
    // poses.push_back(pose);
    // std::ofstream pose_ofs("/home/zhw/avp/hw_ws/src/avp_mapping/poses.bin",
    // std::ios::binary); SaveVector(pose_ofs, poses);
  }

  template <typename T>
  void SaveVector(std::ofstream &ofs, const std::vector<T> &vec) {
    size_t size = vec.size();
    ofs.write(reinterpret_cast<const char *>(&size), sizeof(size));
    ofs.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
  }

  void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
    if (pose_queue_.empty()) {
      std::cout << "pose queue is empty" << std::endl;
      return;
    }
    // 找到与图像时间戳最接近的位姿
    double img_time = msg->header.stamp.toSec();
    size_t closest_idx = 0;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < pose_queue_.size(); ++i) {
      double pose_time = pose_queue_[i].time_;
      double diff = std::abs(pose_time - img_time);

      if (diff < min_diff) {
        min_diff = diff;
        closest_idx = i;
      }
    }
    std::cout << "min_diff " << min_diff << std::endl;
    std::cout << std::fixed << "pose_time " << pose_queue_[closest_idx].time_
              << std::endl;
    std::cout << std::fixed << "img_time " << img_time << std::endl;
    if (min_diff > 0.1) {
      return;
    }
    std::cout << "run" << std::endl;

    TimedPose cur_pose = pose_queue_[closest_idx];

    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    ExtractSlot(gray_image, cur_pose);
  }
  void ExtractSlot(const cv::Mat &img_gray, const TimedPose &pose) {
    cv::Mat slot_img = cv::Mat::zeros(img_gray.size(), img_gray.type());
    for (int i = 0; i < img_gray.rows; ++i) {
      for (int j = 0; j < img_gray.cols; ++j) {
        if (kSlotGray == img_gray.at<uchar>(i, j) ||
            kSlotGray1 == img_gray.at<uchar>(i, j)) {
          slot_img.at<uchar>(i, j) = 254;
        } else if (kArrowGray == img_gray.at<uchar>(i, j)) {
          avp_map_.addSemanticElement(
              SemanticLabel::kArrowLine,
              {ipmPlane2Global(pose, cv::Point2f(j, i))});
        } else if (kDashGray == img_gray.at<uchar>(i, j)) {
          avp_map_.addSemanticElement(
              SemanticLabel::kDashLine,
              {ipmPlane2Global(pose, cv::Point2f(j, i))});
        } else {
        }
      }
    }

    int kernelSize = 15;
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::Mat closed;
    cv::morphologyEx(slot_img, closed, cv::MORPH_CLOSE, kernel);
    auto line_img = closed.clone();

    cv::Mat skel = skeletonize(closed);
    removeIsolatedPixels(skel, 1);
    cv::namedWindow("line_img", cv::WINDOW_NORMAL);
    cv::imshow("line_img", line_img);
    cv::waitKey(1);
    // extract slot lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 50, 50, 50);
    // detect slots
    auto [corners, slot_points] = detectSlot(slot_img, lines);

    // add detected slots to map
    std::vector<Slot> slots(slot_points.size() / 4);
    int index = 0;
    for (auto &slot : slots) {
      slot.corners_[0] = ipmPlane2Global(pose, slot_points[index++]);
      slot.corners_[1] = ipmPlane2Global(pose, slot_points[index++]);
      slot.corners_[2] = ipmPlane2Global(pose, slot_points[index++]);
      slot.corners_[3] = ipmPlane2Global(pose, slot_points[index++]);
      avp_map_.addSlot(slot); // add slot to map
    }
    // update viewer
    viewer_.displayAvpMap(pose, avp_map_, slots);
    viewer_.displayIpmDetection(corners, slot_points, img_gray);
    // 保存地图
    avp_map_.save(std::string(DATASET_PATH) + "/avp_map.bin");
  }
  Eigen::Vector3d ipmPlane2Global(const TimedPose &pose,
                                  const cv::Point2f &ipm_point) {
    Eigen::Vector3d pt_global;
    //////////////////////// TODO: transform ipm pixel to point in global
    //////////////////////////
    Eigen::Vector3d p_ipm;
    p_ipm.x() = -(500 - ipm_point.x) * 0.02;
    p_ipm.y() = (500 - ipm_point.y) * 0.02;
    p_ipm.z() = 0;
    Eigen::Affine3d T_vehicle_ipm;
    T_vehicle_ipm.linear().setIdentity();
    T_vehicle_ipm.translation() = Eigen::Vector3d(0.0, 1.32, 0.0);
    pt_global = pose.R_ * T_vehicle_ipm * p_ipm + pose.t_;
    return pt_global;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping");
  ros::NodeHandle n;
  RosViewer viewer(n);
  Pipeline pipeline(n, viewer);
  ros::spin();
  return 0;
}
