/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/PoseStamped.h>

// project
#include <ek_challenger/data_conversions.hpp>

namespace ek_challenger {

class CreateObstacleActionNode : public BT::AsyncActionNode {
 public:
  CreateObstacleActionNode(const std::string &name,
                           const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::PoseStamped>(
            "pose"),  // pose of the obstacle in the scene
        BT::InputPort<std::string>("object_id"),  // planning scene obstacle id
        BT::InputPort<std::string>("ns"),         // moveit namespace
    };
  }

  BT::NodeStatus tick() override;

  virtual void halt() override;

 private:
  std::atomic_bool _halt_requested;
};

}  // namespace ek_challenger