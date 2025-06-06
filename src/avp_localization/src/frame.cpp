#include "frame.h"

const std::unordered_set<Eigen::Vector3i, Vector3iHash> &
Frame::getSemanticElement(SemanticLabel label) const {
  return semantic_grid_map_.at(label);
}

// add semantic element
void Frame::addSemanticElement(SemanticLabel label, const Eigen::Vector3d &pt) {
  Eigen::Vector3i grid_pt;
  grid_pt.x() = std::round(pt.x() * kPixelScaleInv);
  grid_pt.y() = std::round(pt.y() * kPixelScaleInv);
  grid_pt.z() = std::round(pt.z() * kPixelScaleInv);
  semantic_grid_map_[label].insert(grid_pt);  // like a voxel filter
}

void Frame::clearGridMap() {
  for (auto &it : semantic_grid_map_) {
    it.clear();
  }
}