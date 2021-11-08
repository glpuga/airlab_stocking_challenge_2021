/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <string>
#include <vector>

// ros
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/console.h>
#include <ros/ros.h>

// project
#include <ek_challenger/collision_object_manager.hpp>

namespace ek_challenger {

CollisionObjectManager::CollisionObjectManager(
    const std::vector<std::string> &moveit_namespaces) {
  for (const auto &ns : moveit_namespaces) {
    planning_scene_managers_.push_back(
        std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
            ns));
  }
}

void CollisionObjectManager::addCylinder(const std::string &id,
                                         const geometry_msgs::PoseStamped &pose,
                                         const double radius,
                                         const double height,
                                         const bool is_present) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_RADIUS] = radius*0.95;
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = height;

  geometry_msgs::Pose box_pose;
  box_pose.position = pose.pose.position;
  box_pose.orientation = pose.pose.orientation;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = is_present
                                   ? moveit_msgs::CollisionObject::ADD
                                   : moveit_msgs::CollisionObject::REMOVE;

  known_objects_[id] = collision_object;
}

void CollisionObjectManager::addBox(const std::string &id,
                                    const geometry_msgs::PoseStamped &pose,
                                    const double width, const double height,
                                    const double depth) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = width;
  primitive.dimensions[primitive.BOX_Y] = height;
  primitive.dimensions[primitive.BOX_Z] = depth;

  geometry_msgs::Pose box_pose;
  box_pose.position = pose.pose.position;
  box_pose.orientation = pose.pose.orientation;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = moveit_msgs::CollisionObject::ADD;

  known_objects_[id] = collision_object;
}

void CollisionObjectManager::updatePlanningScene() const {
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  for (const auto &item : known_objects_) {
    collision_objects.push_back(item.second);
  }

  for (const auto &planning_scene_manager : planning_scene_managers_) {
    planning_scene_manager->applyCollisionObjects(collision_objects);
    planning_scene_manager->addCollisionObjects(collision_objects);
  }
}

}  // namespace ek_challenger
