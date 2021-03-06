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
#include <ek_challenger/tomato_can_dimensions.hpp>
#include <ek_challenger/tray_model_impl.hpp>

namespace ek_challenger {

TrayModelImpl::TrayModelImpl(const std::string &name,
                             const geometry_msgs::PoseStamped &pose,
                             const std::vector<std::string> &moveit_namespaces)
    : name_{name}, pose_{pose}, collision_object_manager_{moveit_namespaces} {
  marker_pub_ = ros::NodeHandle("~").advertise<visualization_msgs::Marker>(
      "tomato_can_makers_" + name_, 10);
}

geometry_msgs::PoseStamped TrayModelImpl::trayPose() const { return pose_; }

void TrayModelImpl::clear() {
  std::lock_guard<std::mutex> l{mutex_};
  for (auto &pair : tomato_can_loci_) {
    pair.second.occupied = false;
  }
  updatePlanningScene();

  tomato_can_loci_.clear();
}

std::string TrayModelImpl::addLocus(
    const geometry_msgs::PoseStamped &absolute_pose, const bool occupied,
    const int32_t) {
  std::lock_guard<std::mutex> l{mutex_};
  const auto same_frame_pose = frame_transformer_.transformPoseToFrame(
      absolute_pose, pose_.header.frame_id);

  const auto locus_id = getUniqueId();
  LocusData locus_data;
  locus_data.locus_id = locus_id;

  locus_data.relative_pose.position.x =
      same_frame_pose.pose.position.x - pose_.pose.position.x;
  locus_data.relative_pose.position.y =
      same_frame_pose.pose.position.y - pose_.pose.position.y;
  locus_data.relative_pose.position.z =
      same_frame_pose.pose.position.z - pose_.pose.position.z;

  locus_data.occupied = occupied;

  tomato_can_loci_.emplace(std::make_pair(locus_id, locus_data));
  return locus_id;
}

std::string TrayModelImpl::addLocus(const geometry_msgs::Pose2D &relative_pose,
                                    const bool occupied,
                                    const int32_t priority) {
  geometry_msgs::Pose local_pose;
  local_pose.position.x = relative_pose.x;
  local_pose.position.y = relative_pose.y;
  local_pose.position.z =
      TomatoCanDimensions::height() * 0.5 + tray_to_can_distance_;
  return addLocus(convertRelativeToAbsolute(local_pose), occupied, priority);
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
  return pickLocusWithGivenOccupiedState(from_side, locus_id, true);
}

void TrayModelImpl::setLocusState(const std::string &locus_id,
                                  const bool occupied) {
  std::lock_guard<std::mutex> l{mutex_};
  tomato_can_loci_.at(locus_id).in_use = false;
  tomato_can_loci_.at(locus_id).occupied = occupied;
}

void TrayModelImpl::releaseLocus(const std::string &locus_id) {
  std::lock_guard<std::mutex> l{mutex_};
  tomato_can_loci_.at(locus_id).in_use = false;
}

const geometry_msgs::PoseStamped TrayModelImpl::getLocusPose(
    const std::string &locus_id) const {
  std::lock_guard<std::mutex> l{mutex_};
  const auto &locus = tomato_can_loci_.at(locus_id);
  return convertRelativeToAbsolute(locus.relative_pose);
}

std::string TrayModelImpl::getUniqueId() const {
  static int32_t instance_counter{0};
  return fmt::format("object_{}", instance_counter++);
}

void TrayModelImpl::updateLocalPoses() {
  for (auto &pair : tomato_can_loci_) {
    auto &data = pair.second;
    data.base_link_pose = frame_transformer_.transformPoseToFrame(
        convertRelativeToAbsolute(data.relative_pose), base_link_frame_);
  }
}

std::array<std::set<std::string>, 8>
TrayModelImpl::findApproachDirectionForOccupiedLoci() {
  auto get_bin_index = [](const geometry_msgs::PoseStamped &target,
                          const geometry_msgs::PoseStamped &obstacle) {
    const auto xdiff = obstacle.pose.position.x - target.pose.position.x;
    const auto ydiff = obstacle.pose.position.y - target.pose.position.y;
    const auto angle_deg = (180.0 / 3.1415) * std::atan2(ydiff, xdiff);
    // cheap and dirty to make sure we don't get negative indexes
    const auto bin_index = ((static_cast<int>(angle_deg) + 360) / 45) % 8;
    return bin_index;
  };

  std::array<std::set<std::string>, 8> approach_direction_sets;

  for (const auto &target_pair : tomato_can_loci_) {
    const auto &target_id = target_pair.first;
    const auto &target_pose = target_pair.second.base_link_pose;

    std::array<bool, 8> directions_free_of_obstacles;
    std::for_each(directions_free_of_obstacles.begin(),
                  directions_free_of_obstacles.end(),
                  [](bool &item) { item = true; });

    for (const auto &obstacle_pair : tomato_can_loci_) {
      const auto &obstacle_id = obstacle_pair.first;
      const auto &obstacle_pose = obstacle_pair.second.base_link_pose;

      // don't check an object with itself
      if (target_id == obstacle_id) {
        continue;
      }

      const auto bin_index = get_bin_index(target_pose, obstacle_pose);

      if (obstacle_pair.second.occupied) {
        directions_free_of_obstacles[bin_index] = false;
        // also mark one before and one after, since those are very likely
        // partially obstructed
        directions_free_of_obstacles[(bin_index + 1) % 8] = false;
        directions_free_of_obstacles[(bin_index + 7) % 8] = false;
      }
    }

    // add object id to the sets for free approach directions
    for (size_t i = 0; i < directions_free_of_obstacles.size(); ++i) {
      // if a given approach direction is free add the object id to the
      // corresponding set
      if (directions_free_of_obstacles[i]) {
        approach_direction_sets[i].insert(target_id);
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

  for (const auto &outer_pair : tomato_can_loci_) {
    const auto &outer_id = outer_pair.first;
    const auto &outer_pose = outer_pair.second.base_link_pose;

    std::array<bool, 8> directions_free_of_obstacles;
    std::for_each(directions_free_of_obstacles.begin(),
                  directions_free_of_obstacles.end(),
                  [](bool &item) { item = true; });

    for (const auto &obstacle_pair : tomato_can_loci_) {
      const auto &obstacle_id = obstacle_pair.first;
      const auto &obstacle_pose = obstacle_pair.second.base_link_pose;

      // don't check an object with itself
      if (outer_id == obstacle_id) {
        continue;
      }

      const auto bin_index = get_bin_index(outer_pose, obstacle_pose);

      if (!obstacle_pair.second.occupied) {
        directions_free_of_obstacles[bin_index] = false;
      }
    }

    // add object id to the sets for free approach directions
    for (size_t i = 0; i < directions_free_of_obstacles.size(); ++i) {
      // if a given approach direction is free add the object id to the
      // corresponding set
      if (directions_free_of_obstacles[i]) {
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
      id = pickARandomObjectIdFromBins({2, 3, 4}, approach_sets);
    } else if ((from_side == Side::RIGHT) && occupied) {
      id = pickARandomObjectIdFromBins({3, 4, 5}, approach_sets);
    } else if ((from_side == Side::LEFT) && !occupied) {
      id = pickARandomObjectIdFromBins({0, 7, 6}, approach_sets);
    } else if ((from_side == Side::RIGHT) && !occupied) {
      id = pickARandomObjectIdFromBins({0, 1, 7}, approach_sets);
    }

    if (id == "") {
      // there are no ids that fit the request
      return false;
    } else if ((tomato_can_loci_[id].in_use == false) &&
               (tomato_can_loci_[id].occupied == occupied)) {
      // we found one!
      locus_id = id;
      tomato_can_loci_.at(locus_id).in_use = true;
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

  for (const auto &outer_pair : tomato_can_loci_) {
    const auto &outer_id = outer_pair.first;

    if (outer_pair.second.occupied == occupied) {
      auto base_link_position = outer_pair.second.relative_pose.position;
      base_link_position.x += 1.0;  // hackish way to keep the fill order in the
                                    // backgraty from front to back
      const auto ref_depth =
          std::sqrt((base_link_position.x * base_link_position.x) +
                    (base_link_position.y * base_link_position.y)) -
          base_link_position.z * 0.01;

      if (get_max) {
        if (current_x_value < ref_depth) {
          locus_id = outer_id;
          current_x_value = ref_depth;
        }
      } else {
        if (current_x_value > ref_depth) {
          locus_id = outer_id;
          current_x_value = ref_depth;
        }
      }
    }
  }

  // if we didn't find any, return failure
  if (locus_id == "") {
    return false;
  }

  tomato_can_loci_.at(locus_id).in_use = true;

  return true;
}

std::string TrayModelImpl::pickARandomObjectIdFromBins(
    const std::set<int32_t> &indexes,
    const std::array<std::set<std::string>, 8> &directions_free_of_obstacles) {
  // merge all valid sets into one larger set
  std::set<std::string> full_set;
  for (const auto index : indexes) {
    const auto &partial_bin = directions_free_of_obstacles[index];
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

  publishMarkers();
}

void TrayModelImpl::updateSceneAddingFrame(CollisionObjectManager &) {}

void TrayModelImpl::updateSceneAddingCans() {
  for (const auto &obstacle_pair : tomato_can_loci_) {
    const auto &id = obstacle_pair.first;
    const auto &pose =
        convertRelativeToAbsolute(obstacle_pair.second.relative_pose);
    collision_object_manager_.addCylinder(
        id, pose, TomatoCanDimensions::width() * 0.5,
        TomatoCanDimensions::height(), obstacle_pair.second.occupied);

    if (!obstacle_pair.second.occupied && obstacle_pair.second.in_use) {
      // {
      // auto base_pose = pose;
      // base_pose.pose.position.z -= TomatoCanDimensions::height() * 0.7;
      // collision_object_manager_.addCylinder(
      //     "target", base_pose, TomatoCanDimensions::width(),
      //     TomatoCanDimensions::height() * 0.05, true);
      // }
      // {
      //   auto base_pose = pose;
      //   base_pose.pose.position.z += TomatoCanDimensions::height() * 0.5;
      //   collision_object_manager_.addCylinder(
      //       "target", base_pose, TomatoCanDimensions::width() * 0.5,
      //       TomatoCanDimensions::height() * 2, true);
      // }
      {
        collision_object_manager_.addCylinder(
            "target", pose, TomatoCanDimensions::width() * 0.5,
            TomatoCanDimensions::height() * 0.8, true);
      }
    }
  }
}

geometry_msgs::PoseStamped TrayModelImpl::convertRelativeToAbsolute(
    const geometry_msgs::Pose &relative_pose) const {
  auto absolute_pose = pose_;
  absolute_pose.pose.position.x += relative_pose.position.x;
  absolute_pose.pose.position.y += relative_pose.position.y;
  absolute_pose.pose.position.z += relative_pose.position.z;
  return absolute_pose;
}

void TrayModelImpl::publishMarkers() const {
  int32_t id_cnt{0};
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(marker);

  for (const auto &locus : tomato_can_loci_) {
    const auto &pose = convertRelativeToAbsolute(locus.second.relative_pose);
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name_;
    marker.id = ++id_cnt;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;

    if (locus.second.occupied) {
      marker.color.a = 0.6;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      marker.color.a = 0.6;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }

    if (locus.second.in_use) {
      marker.scale.x = TomatoCanDimensions::width() * 1.1;
      marker.scale.y = TomatoCanDimensions::depth() * 1.1;
      marker.scale.z = TomatoCanDimensions::height() * 1.1;
      marker.color.a = 0.9;
    } else {
      marker.scale.x = TomatoCanDimensions::width();
      marker.scale.y = TomatoCanDimensions::depth();
      marker.scale.z = TomatoCanDimensions::height();
    }

    marker_pub_.publish(marker);
  }
}

}  // namespace ek_challenger
