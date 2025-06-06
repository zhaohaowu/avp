#pragma once
#include <functional>
#include <mutex>
#include <thread>

#include "map.h"

class MapViewer {
 public:
  ~MapViewer() {
    count_ = -1;
    thread_.join();
  }
  MapViewer(const Map &avp_map) {
    bounding_box_ = avp_map.getBoundingBox();
    if (bounding_box_.isEmpty()) {  // empty map
      bounding_box_.extend(Eigen::Vector3d(0, 0, 0));
    }
    Eigen::Vector3d bl = bounding_box_.min() + Eigen::Vector3d(-1, -1, -1);
    Eigen::Vector3d tr = bounding_box_.max() + Eigen::Vector3d(1, 1, 1);
    int width = (tr.x() - bl.x()) * kPixelScaleInv;
    int height = (tr.y() - bl.y()) * kPixelScaleInv;
    map_ = cv::Mat(height, width, CV_8UC3, cv::Vec3b(91, 91, 91));

    to_map_pixel_ = [=](const Eigen::Vector3d &pt) {
      return cv::Point2f((pt.x() - bl.x()) * kPixelScaleInv,
                         height - (pt.y() - bl.y()) * kPixelScaleInv);
    };

    offset_ = to_map_pixel_(Eigen::Vector3d(0, 0, 0));
    drawGrid(map_, avp_map.getSemanticElement(SemanticLabel::kSlot),
             cv::Vec3b(kSlotGray, kSlotGray, kSlotGray));
    drawGrid(map_, avp_map.getSemanticElement(SemanticLabel::kDashLine),
             cv::Vec3b(kDashGray, kDashGray, kDashGray));
    drawGrid(map_, avp_map.getSemanticElement(SemanticLabel::kArrowLine),
             cv::Vec3b(kArrowGray, kArrowGray, kArrowGray));
    thread_ = std::thread([=] { run(); });
  }

  void showFrame(const Frame &frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    frame_ = frame;
    ++count_;
  }

 private:
  void drawGrid(const cv::Mat &map,
                const std::unordered_set<Eigen::Vector3i, Vector3iHash> &grid,
                cv::Vec3b color) const {
    for (const auto &pt : grid) {
      cv::circle(map, offset_ + cv::Point2f(pt.x(), -pt.y()), 1, color, 1);
    }
  }

  void run() {
    int cnt = count_;
    cv::Mat show = map_.clone();
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::resizeWindow("map", 1600, 1000);
    while (cnt > 0) {
      if (cnt != count_) {
        cnt = count_;
        std::unique_lock<std::mutex> lock(mutex_);
        auto frame = std::move(frame_);
        lock.unlock();
        auto pos = to_map_pixel_(frame.t_update);
        if (!trajectory_.empty()) {
          cv::line(map_, trajectory_.back(), pos, cv::Scalar(255, 0, 155), 10);
        }
        trajectory_.push_back(pos);
        show = map_.clone();
        drawGrid(show, frame.getSemanticElement(SemanticLabel::kSlot),
                 cv::Vec3b(0, 0, 255));
        drawGrid(show, frame.getSemanticElement(SemanticLabel::kDashLine),
                 cv::Vec3b(0, 255, 0));
        drawGrid(show, frame.getSemanticElement(SemanticLabel::kArrowLine),
                 cv::Vec3b(255, 0, 0));
      }
      cv::Mat rotated_show;
      cv::rotate(show, rotated_show, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::imshow("map", rotated_show);
      cv::waitKey(50);
    }
    cv::destroyAllWindows();
  }

  std::mutex mutex_;
  int count_ = 1;
  Frame frame_;
  std::thread thread_;

  cv::Mat map_;
  Eigen::AlignedBox3d bounding_box_;
  cv::Point2f offset_;
  std::vector<cv::Point2f> trajectory_;
  std::function<cv::Point2f(const Eigen::Vector3d &)> to_map_pixel_;
};