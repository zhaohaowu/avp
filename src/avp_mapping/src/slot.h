#pragma once
#include <array>
#include <tuple>
#include <vector>
#include <unordered_set>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace {
// parameters for slot
constexpr int kIPMImgHeight = 1000;
constexpr int kIPMImgWidth = 1000;

constexpr int kShortLinePixelDistance = 210;
constexpr int kLongLinePixelDistance = 320;
constexpr int kLongLineDetectPixelNumberMin = 200;
constexpr int kShortLineDetectPixelNumberMin = 50;

constexpr int kNeighborDistance = 100; // 两车位点小于这个距离视为同一个车位点
constexpr double kPixelScale = 0.02;
constexpr double kPixelScaleInv = 1.0 / kPixelScale;

const uchar kDashGray = 127;
const uchar kArrowGray = 226;
const uchar kSlotGray = 153;
const uchar kSlotGray1 = 152;

constexpr double kToRad = M_PI / 180.0;
constexpr double kToDeg = 180.0 / M_PI;
} // end of namespace

enum SemanticLabel {
  kDashLine = 0,
  kArrowLine,
  kTotalLabelNum // flag that indicate label num
};

struct Slot {
  Eigen::Vector3d corners_[4];
};

// slot cornerPoints
class CornerPoint {
public:
  CornerPoint() : CornerPoint({0,0,0}) {}
  explicit CornerPoint(const Eigen::Vector3d &pt) : center_(pt), count_(1) {}
  // return true if the point is absorbed (distance < dist)
  bool absorb(const Eigen::Vector3d &pt, const double dist) {
    if ((pt - center()).norm() < dist) {
      center_ += pt;
      ++count_;
      return true;
    }
    return false;
  }
  Eigen::Vector3d center() const {
    return center_ / count_;
  }
private:
  Eigen::Vector3d center_;
  unsigned int count_;
};
// image processing utils for slot detection
cv::Mat skeletonize(const cv::Mat &img);
void removeIsolatedPixels(cv::Mat &src, int min_neighbors = 1);
bool isSlotLongLine(const cv::Mat &line_img, const cv::Point2f &start,
                    const cv::Point2f &dir);
bool isSlotShortLine(const cv::Point2f &point1, const cv::Point2f &point2,
                     const cv::Mat &mask);
std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> detectSlot(
    const cv::Mat &slot_img, const std::vector<cv::Vec4i> &lines);
// IO method for data serialization and deserialization
namespace io{
template <typename T>
void loadVector(std::ifstream &ifs, std::vector<T> &vec) {
  size_t size;
  ifs.read(reinterpret_cast<char *>(&size), sizeof(size));
  vec.resize(size);
  ifs.read(reinterpret_cast<char *>(vec.data()), size * sizeof(T));
}
template <typename T>
void saveVector(std::ofstream &ofs, const std::vector<T> &vec) {
  size_t size = vec.size();
  ofs.write(reinterpret_cast<const char *>(&size), sizeof(size));
  ofs.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
}
}