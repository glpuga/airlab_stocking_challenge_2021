/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <map>
#include <string>

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

// MTC
#include <eigen_conversions/eigen_msg.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

namespace ek_challenger {
class PickPlaceTask {
 public:
  PickPlaceTask(const std::string &side);

  bool build(const std::string &object_name,
             const geometry_msgs::PoseStamped &target_pose,
             const bool move_to_home = false);

  bool plan();

  bool execute();

 private:
  const std::string task_name_{"pick_and_place_task"};
  const std::string world_frame_{"map"};

  const double approach_object_min_dist_{0.15};
  const double approach_object_max_dist_{0.25};
  const double lift_object_min_dist_{0.01};
  const double lift_object_max_dist_{0.10};

  const double connect_planning_timeout_{25};

  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string hand_frame_;
  std::string eef_name_;

  geometry_msgs::PoseStamped grasp_frame_transform_;

  std::map<std::string, double> open_hand_joint_values_;
  std::map<std::string, double> closed_hand_joint_values_;
  std::map<std::string, double> resting_arm_joint_poses_;

  moveit::task_constructor::TaskPtr task_;
};

}  // namespace ek_challenger
