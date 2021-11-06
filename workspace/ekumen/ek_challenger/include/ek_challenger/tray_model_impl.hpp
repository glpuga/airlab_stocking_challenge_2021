/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <vector>

// third-party
#include <fmt/format.h>

// ros
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

// project
#include <ek_challenger/collision_object_manager.hpp>
#include <ek_challenger/ros_frame_transformer.hpp>
#include <ek_challenger/tray_model_interface.hpp>

namespace ek_challenger {

class TrayModelImpl : public TrayModelInterface {
 public:
  TrayModelImpl(const std::string &name, const geometry_msgs::PoseStamped &pose,
                const std::vector<std::string> &moveit_namespaces);

  geometry_msgs::PoseStamped trayPose() const override;

  void clear() override;

  std::string addLocus(const geometry_msgs::PoseStamped &absolute_pose,
                       const bool occupied, const int32_t) override;

  std::string addLocus(const geometry_msgs::Pose2D &relative_pose,
                       const bool occupied, const int32_t priority) override;

  bool pickEmptyLocus(const Side &from_side, std::string &locus_id) override;

  bool pickOccupiedLocus(const Side &from_side, std::string &locus_id) override;

  void setLocusState(const std::string &locus_id, const bool occupied) override;

  const geometry_msgs::PoseStamped getLocusPose(
      const std::string &locus_id) const override;

  void releaseLocus(const std::string &locus_id) override;

  void updatePlanningScene() override;

 protected:
  std::string getUniqueId() const;

  virtual void updateSceneAddingFrame(CollisionObjectManager &);

 private:
  struct LocusData {
    std::string locus_id;
    geometry_msgs::Pose relative_pose;
    bool occupied{false};
    bool in_use{false};

    geometry_msgs::PoseStamped base_link_pose{};
  };

  const std::string name_;
  const std::string base_link_frame_{"base_link"};
  const int32_t max_allocation_attempts_{20};

  const double tray_to_can_distance_{0.00};  // was 0.005

  const geometry_msgs::PoseStamped pose_;
  CollisionObjectManager collision_object_manager_;

  mutable std::mutex mutex_;

  std::random_device random_seed_;
  std::mt19937 random_generator_{random_seed_()};

  ROSFrameTransformer frame_transformer_;

  std::map<std::string, LocusData> tomato_can_loci_;

  ros::Publisher marker_pub_;

  void updateLocalPoses();

  std::array<std::set<std::string>, 8> findApproachDirectionForOccupiedLoci();
  std::array<std::set<std::string>, 8> findApproachDirectionForEmptyLoci();

  bool pickLocusWithGivenOccupiedState(const Side &from_side,
                                       std::string &locus_id,
                                       const bool occupied);

  bool pickLocusWithGivenOccupiedStateByDepth(std::string &locus_id,
                                              const bool occupied,
                                              const bool get_max);

  std::string pickARandomObjectIdFromBins(
      const std::set<int32_t> &indexes,
      const std::array<std::set<std::string>, 8> &bins);

  void updateSceneAddingCans();

  geometry_msgs::PoseStamped convertRelativeToAbsolute(
      const geometry_msgs::Pose &relative_pose) const;

  void publishMarkers() const;
};

}  // namespace ek_challenger
