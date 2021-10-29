/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <map>
#include <memory>
#include <mutex>
#include <string>

// ros
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

namespace ek_challenger {

class CollisionObjectManager {
 public:
  CollisionObjectManager(const std::vector<std::string> &moveit_namespaces);

  void addCylinder(const std::string &id,
                   const geometry_msgs::PoseStamped &pose, const double radius,
                   const double height, const bool is_present);

  void addBox(const std::string &id, const geometry_msgs::PoseStamped &pose,
              const double width, const double height, const double depth);

  void updatePlanningScene() const;

 private:
  using PlanningSceneManagerPtr =
      std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>;

  std::map<std::string, moveit_msgs::CollisionObject> known_objects_;

  std::vector<PlanningSceneManagerPtr> planning_scene_managers_;


};
}  // namespace ek_challenger
