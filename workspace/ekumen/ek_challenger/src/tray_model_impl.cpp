/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

// third-party
#include <ros/console.h>

// project
#include <ek_challenger/tray_model_impl.hpp>

namespace ek_challenger {

TrayModelImpl::TrayModelImpl(const geometry_msgs::PoseStamped &pose,
                             const std::vector<std::string> &moveit_namespaces)
    : pose_{pose}, collision_object_manager_{moveit_namespaces} {}

geometry_msgs::PoseStamped TrayModelImpl::trayPose() const { return pose_; }

std::string TrayModelImpl::addLocus(
    const geometry_msgs::PoseStamped &absolute_pose, const bool occupied,
    const int32_t) {
  std::lock_guard<std::mutex> l{mutex_};
  const auto same_frame_pose = frame_transformer_.transformPoseToFrame(
      absolute_pose, pose_.header.frame_id);

  const auto locus_id = getUniqueId();
  LocusData locus_data;
  locus_data.locus_id = locus_id;

  locus_data.relative_pose.x =
      same_frame_pose.pose.position.x - pose_.pose.position.x;
  locus_data.relative_pose.y =
      same_frame_pose.pose.position.y - pose_.pose.position.y;

  locus_data.occupied = occupied;

  loci_.emplace(std::make_pair(locus_id, locus_data));
  return locus_id;
}

std::string TrayModelImpl::addLocus(const geometry_msgs::Pose2D &relative_pose,
                                    const bool occupied,
                                    const int32_t priority) {
  return addLocus(convertRelativeToAbsolute(relative_pose), occupied, priority);
}

bool TrayModelImpl::pickEmptyLocus(const Side &from_side,
                                   std::string &locus_id) {
  std::lock_guard<std::mutex> l{mutex_};
  (void)from_side;
  return pickLocusWithGivenOccupiedStateByDepth(locus_id, false, true);
}

bool TrayModelImpl::pickOccupiedLocus(const Side &from_side,
                                      std::string &locus_id) {
  std::lock_guard<std::mutex> l{mutex_};
  (void)from_side;
  return pickLocusWithGivenOccupiedStateByDepth(locus_id, true, false);
}

void TrayModelImpl::setLocusState(const std::string &locus_id,
                                  const bool occupied) {
  std::lock_guard<std::mutex> l{mutex_};
  loci_.at(locus_id).in_use = false;
  loci_.at(locus_id).occupied = occupied;
}

void TrayModelImpl::releaseLocus(const std::string &locus_id) {
  std::lock_guard<std::mutex> l{mutex_};
  loci_.at(locus_id).in_use = false;
}

const geometry_msgs::PoseStamped TrayModelImpl::getLocusPose(
    const std::string &locus_id) const {
  std::lock_guard<std::mutex> l{mutex_};
  const auto &locus = loci_.at(locus_id);
  return convertRelativeToAbsolute(locus.relative_pose);
}

std::string TrayModelImpl::getUniqueId() const {
  static int32_t instance_counter{0};
  return fmt::format("object_{}", instance_counter++);
}

void TrayModelImpl::updateLocalPoses() {
  for (auto &pair : loci_) {
    auto &data = pair.second;
    data.base_link_pose = frame_transformer_.transformPoseToFrame(
        convertRelativeToAbsolute(data.relative_pose), base_link_frame_);
  }
}

std::array<std::set<std::string>, 8>
TrayModelImpl::findApproachDirectionForOccupiedLoci() {
  auto get_bin_index = [](const geometry_msgs::PoseStamped &inner,
                          const geometry_msgs::PoseStamped &outer) {
    const auto xdiff = outer.pose.position.x - inner.pose.position.x;
    const auto ydiff = outer.pose.position.y - inner.pose.position.y;
    const auto angle_deg = (180.0 / 3.1415) * std::atan2(ydiff, xdiff);
    // cheap and dirty to make sure we don't get negative indexes
    const auto bin_index = ((static_cast<int>(angle_deg) + 360) / 45) % 8;
    return bin_index;
  };

  std::array<std::set<std::string>, 8> approach_direction_sets;

  for (const auto &outer_pair : loci_) {
    const auto &outer_id = outer_pair.first;
    const auto &outer_pose = outer_pair.second.base_link_pose;

    std::array<bool, 8> bins;
    std::for_each(bins.begin(), bins.end(), [](bool &item) { item = true; });

    for (const auto &inner_pair : loci_) {
      const auto &inner_id = inner_pair.first;
      const auto &inner_pose = inner_pair.second.base_link_pose;

      // don't check an object with itself
      if (outer_id == inner_id) {
        continue;
      }

      const auto bin_index = get_bin_index(outer_pose, inner_pose);

      if (inner_pair.second.occupied) {
        bins[bin_index] = false;
      }
    }

    // add object id to the sets for free approach directions
    for (size_t i = 0; i < bins.size(); ++i) {
      // if a given approach direction is free add the object id to the
      // corresponding set
      if (bins[i]) {
        approach_direction_sets[i].insert(outer_id);
      }
    }
  }

  return approach_direction_sets;
}

std::array<std::set<std::string>, 8>
TrayModelImpl::findApproachDirectionForEmptyLoci() {
  auto get_bin_index = [](const geometry_msgs::PoseStamped &inner,
                          const geometry_msgs::PoseStamped &outer) {
    const auto xdiff = outer.pose.position.x - inner.pose.position.x;
    const auto ydiff = outer.pose.position.y - inner.pose.position.y;
    const auto angle_deg = (180.0 / 3.1415) * std::atan2(ydiff, xdiff);
    // cheap and dirty to make sure we don't get negative indexes
    const auto bin_index = ((static_cast<int>(angle_deg) + 360) / 45) % 8;
    return bin_index;
  };

  std::array<std::set<std::string>, 8> approach_direction_sets;

  for (const auto &outer_pair : loci_) {
    const auto &outer_id = outer_pair.first;
    const auto &outer_pose = outer_pair.second.base_link_pose;

    std::array<bool, 8> bins;
    std::for_each(bins.begin(), bins.end(), [](bool &item) { item = true; });

    for (const auto &inner_pair : loci_) {
      const auto &inner_id = inner_pair.first;
      const auto &inner_pose = inner_pair.second.base_link_pose;

      // don't check an object with itself
      if (outer_id == inner_id) {
        continue;
      }

      const auto bin_index = get_bin_index(outer_pose, inner_pose);

      if (!inner_pair.second.occupied) {
        bins[bin_index] = false;
      }
    }

    // add object id to the sets for free approach directions
    for (size_t i = 0; i < bins.size(); ++i) {
      // if a given approach direction is free add the object id to the
      // corresponding set
      if (bins[i]) {
        approach_direction_sets[i].insert(outer_id);
      }
    }
  }

  return approach_direction_sets;
}

bool TrayModelImpl::pickLocusWithGivenOccupiedState(const Side &from_side,
                                                    std::string &locus_id,
                                                    const bool occupied) {
  updateLocalPoses();

  std::array<std::set<std::string>, 8> approach_sets;

  if (occupied) {
    approach_sets = findApproachDirectionForOccupiedLoci();
  } else {
    approach_sets = findApproachDirectionForEmptyLoci();
  }

  std::string id;
  for (int32_t attempt = 0; attempt < max_allocation_attempts_; ++attempt) {
    if ((from_side == Side::LEFT) && occupied) {
      id = pickARandomObjectIdFromBins({1, 2, 3}, approach_sets);
    } else if ((from_side == Side::RIGHT) && occupied) {
      id = pickARandomObjectIdFromBins({4, 5, 6}, approach_sets);
    } else if ((from_side == Side::LEFT) && !occupied) {
      id = pickARandomObjectIdFromBins({0, 7, 6}, approach_sets);
    } else if ((from_side == Side::RIGHT) && !occupied) {
      id = pickARandomObjectIdFromBins({0, 1, 7}, approach_sets);
    }

    if (id == "") {
      // there are no ids that fit the request
      return false;
    } else if ((loci_[id].in_use == false) &&
               (loci_[id].occupied == occupied)) {
      // we found one!
      locus_id = id;
      loci_.at(locus_id).in_use = true;
      return true;
    }

    // else we keep trying.
  }

  // if we exited the loop without finding anything, we fail
  return false;
}

bool TrayModelImpl::pickLocusWithGivenOccupiedStateByDepth(
    std::string &locus_id, const bool occupied, const bool get_max) {
  updateLocalPoses();

  double current_x_value{10000.0 * (get_max ? -1.0 : 1.0)};

  locus_id = "";

  for (const auto &outer_pair : loci_) {
    const auto &outer_id = outer_pair.first;

    if (outer_pair.second.occupied == occupied) {
      if (get_max) {
        if (current_x_value < outer_pair.second.relative_pose.x) {
          locus_id = outer_id;
          current_x_value = outer_pair.second.relative_pose.x;
        }
      } else {
        if (current_x_value > outer_pair.second.relative_pose.x) {
          locus_id = outer_id;
          current_x_value = outer_pair.second.relative_pose.x;
        }
      }
    }
  }

  // if we didn't find any, return failure
  if (locus_id == "") {
    return false;
  }

  return true;
}

std::string TrayModelImpl::pickARandomObjectIdFromBins(
    const std::set<int32_t> &indexes,
    const std::array<std::set<std::string>, 8> &bins) {
  // merge all valid sets into one larger set
  std::set<std::string> full_set;
  for (const auto index : indexes) {
    const auto &partial_bin = bins[index];
    for (const auto &id : partial_bin) {
      full_set.insert(id);
    }
  }

  // create a vector from the set
  std::vector<std::string> candidates{full_set.begin(), full_set.end()};

  // if there are no available options, return empty
  if (candidates.size() == 0) {
    return "";
  }

  // pick one at random from the set
  std::uniform_int_distribution<> distribution(0, candidates.size() - 1);
  const auto random_index = distribution(random_generator_);

  return candidates[random_index];
}

void TrayModelImpl::updatePlanningScene() {
  updateSceneAddingCans();
  updateSceneAddingFrame(collision_object_manager_);
  collision_object_manager_.updatePlanningScene();
}

void TrayModelImpl::updateSceneAddingFrame(CollisionObjectManager &) {}

void TrayModelImpl::updateSceneAddingCans() {
  for (const auto &inner_pair : loci_) {
    const auto &id = inner_pair.first;
    const auto &pose =
        convertRelativeToAbsolute(inner_pair.second.relative_pose);
    collision_object_manager_.addCylinder(id, pose, can_radius_, can_height_,
                                          inner_pair.second.occupied);
  }
}

geometry_msgs::PoseStamped TrayModelImpl::convertRelativeToAbsolute(
    const geometry_msgs::Pose2D &relative_pose) const {
  auto absolute_pose = pose_;
  absolute_pose.pose.position.x += relative_pose.x;
  absolute_pose.pose.position.y += relative_pose.y;
  absolute_pose.pose.position.z += 0.5 * can_height_ + tray_to_can_distance_;
  return absolute_pose;
}

}  // namespace ek_challenger
