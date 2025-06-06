#include "map.h"

std::vector<Slot> Map::getAllSlots() const {
  std::vector<Slot> all_slots(slots_.size());
  for (int i = 0; i < slots_.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      all_slots[i].corners_[j] = corner_points_[slots_[i][j]].center();
    }
  }
  return std::move(all_slots);
}

const std::unordered_set<Eigen::Vector3i, Map::Vector3iHash>& Map::getSemanticElement(SemanticLabel label) const {
  return semantic_grid_map_.at(label);
}

// add semantic element
void Map::addSemanticElement(SemanticLabel label, const Eigen::Vector3d &pt) {
  Eigen::Vector3i grid_pt;
  grid_pt.x() = std::round(pt.x() * kPixelScaleInv);
  grid_pt.y() = std::round(pt.y() * kPixelScaleInv);
  grid_pt.z() = std::round(pt.z() * kPixelScaleInv);
  semantic_grid_map_[label].insert(grid_pt); // like a voxel filter
}

// add slot to map
void Map::addSlot(const Slot &new_slot) {
  // add slot corner to map
  SlotIndex new_slot_index;
  Eigen::Vector3d new_slot_center = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    new_slot_index[i] = addSlotCorner(new_slot.corners_[i]);
    new_slot_center += new_slot.corners_[i];
  }
  new_slot_center *= 0.25;
  // add to map if new slot
  bool isNewSlot = true;
  Eigen::Vector3d slot_center;
  for (const auto slot_index : slots_) {
    slot_center = Eigen::Vector3d::Zero();
    for (int j = 0; j < 4; ++j) {
      slot_center += corner_points_[slot_index[j]].center();
    }
    slot_center *= 0.25;
    if ((slot_center- new_slot_center).norm() < 1.0) {
      isNewSlot = false;
      break;
    }
  }
  if (isNewSlot) {
    slots_.push_back(new_slot_index);
  }
}

size_t Map::addSlotCorner(const Eigen::Vector3d &pt) {
  size_t index = corner_points_.size();
  for (int i = 0; i < corner_points_.size(); ++i) {
    if (corner_points_[i].absorb(pt, kNeighborDistance * kPixelScale)) {
      index = i;
      break;
    }
  }
  if (corner_points_.size() == index) {
    corner_points_.emplace_back(pt);
    bounding_box_.extend(pt);
  }
  return index;
}

void Map::save(const std::string &filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs.is_open()){
    std::cerr << "Unable to open map file: " << filename << std::endl;
    return;
  }

  io::saveVector(ofs, corner_points_);
  io::saveVector(ofs, slots_);
  std::cout << "save corners: " << corner_points_.size() << std::endl;
  std::cout << "save slots: " << slots_.size() << std::endl;

  std::vector<Eigen::Vector3i> semantic_grid;
  for (int i = 0; i < SemanticLabel::kTotalLabelNum; ++i) {
    semantic_grid.assign(semantic_grid_map_[i].begin(), semantic_grid_map_[i].end());
    io::saveVector(ofs, semantic_grid);
    std::cout << "save label_" << i << ": " << semantic_grid.size() << std::endl;
  }
  ofs.close();
  std::cout << "avp map saved to " << filename << std::endl;
}

void Map::load(const std::string &filename) {
  if (!corner_points_.empty()){
    std::cerr << "map have already loaded, fail to load map file: " << filename << std::endl;
    return;
  }
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs.is_open()){
    std::cerr << "Unable to load map file: " << filename << std::endl;
    return;
  }
  io::loadVector(ifs, corner_points_);
  io::loadVector(ifs, slots_);
  for(const auto &corner : corner_points_){
    bounding_box_.extend(corner.center()); // extend bounding box
  }
  std::cout << "load corners: " << corner_points_.size() << std::endl;
  std::cout << "load slots: " << slots_.size() << std::endl;

  std::vector<Eigen::Vector3i> semantic_grid;
  for (int i = 0; i < SemanticLabel::kTotalLabelNum; ++i) {
    io::loadVector(ifs, semantic_grid);
    semantic_grid_map_[i].insert(semantic_grid.begin(), semantic_grid.end());
    std::cout << "load label_" << i << ": " << semantic_grid.size() << std::endl;
  }
  ifs.close();
  std::cout << "avp map loaded from " << filename << std::endl;
}