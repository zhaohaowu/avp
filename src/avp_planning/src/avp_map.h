#pragma once
#include "slot.h"

struct TimedPose {
    double time_;
    Eigen::Vector3d t_;
    Eigen::Quaterniond R_;
};

class AvpMap {
  using SlotIndex = std::array<size_t, 4>;

 public:
  explicit AvpMap() : semantic_grid_map_(SemanticLabel::kTotalLabelNum) {}

  Eigen::AlignedBox3d getBoundingBox() const { return bounding_box_; }
  std::vector<Slot> getAllSlots() const;
  const std::unordered_set<Eigen::Vector3i, Vector3iHash> &getSemanticElement(
      SemanticLabel label) const;

  void addSemanticElement(SemanticLabel label, const Eigen::Vector3d &pt);
  void addSlot(const Slot &new_slot);

  void save(const std::string &filename) const;
  void load(const std::string &filename);

  void discretizeLine(const std::vector<Slot> &slots);

 private:
  // return the index of the corner point
  size_t addSlotCorner(const Eigen::Vector3d &pt);

  std::vector<CornerPoint> corner_points_;  // slot corners
  std::vector<SlotIndex> slots_;  // index of slot points, 4 points for a slot
  std::vector<std::unordered_set<Eigen::Vector3i, Vector3iHash>>
      semantic_grid_map_;             // grid of semantic elements
  Eigen::AlignedBox3d bounding_box_;  // bounding box of slot corners
};

template <typename T>
T GetYaw(const Eigen::Quaternion<T> &rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}
