#pragma once
#include "slot.h"

class Frame {
 public:
  explicit Frame() : semantic_grid_map_(SemanticLabel::kTotalLabelNum) {}
  const std::unordered_set<Eigen::Vector3i, Vector3iHash>& getSemanticElement(
      SemanticLabel label) const;
  void addSemanticElement(SemanticLabel label, const Eigen::Vector3d& pt);
  void clearGridMap();
  std::vector<std::unordered_set<Eigen::Vector3i, Vector3iHash>>
  getSemanticGridMap() {
    return semantic_grid_map_;
  };

  Eigen::Affine3d T_world_ipm_;  // init_pose
  Eigen::Vector3d t_update; // for viewer
 private:
  std::vector<std::unordered_set<Eigen::Vector3i, Vector3iHash>>
      semantic_grid_map_;  // grid of semantic elements
};
