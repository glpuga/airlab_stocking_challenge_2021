/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <atomic>
#include <mutex>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/PoseStamped.h>

class PickAndPlaceActionNode : public BT::AsyncActionNode {
 public:
  PickAndPlaceActionNode(const std::string &name,
                         const BT::NodeConfiguration &config,
                         const std::string &side,
                         const std::string &moveit_namespace)
      : AsyncActionNode(name, config),
        side_{side},
        moveit_namespace_{moveit_namespace} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>(
            "object_id"), /* planning scene object to pick */
        BT::InputPort<geometry_msgs::PoseStamped>(
            "target_pose"), /* target pose */
        BT::InputPort<geometry_msgs::PoseStamped>(
            "flat_hand_mode"), /* flat_hand_mode */
    };
  }

  BT::NodeStatus tick() override;

  void halt() override;

 private:
  const std::string side_;
  const std::string moveit_namespace_;

  std::atomic_bool halt_requested_{false};
};
