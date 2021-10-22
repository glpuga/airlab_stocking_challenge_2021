/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <map>
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/arm_joints_pose.hpp>

namespace ek_challenger {

class GetArmJointsForPoseActionNode : public BT::AsyncActionNode {
 public:
  GetArmJointsForPoseActionNode(const std::string &name,
                                const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("pose"),
        BT::OutputPort<ArmJointsPose>("joints_pose"),
    };
  }

  BT::NodeStatus tick() override;

  virtual void halt() override;

 private:
  std::map<std::string, ArmJointsPose> stored_poses_ = {
      {"REST", {-63, 84, 156, 98, -90, 80, 0}},
      {"EXTENDED", {0, 0, 0, 0, 0, 0, 0}},
      {"UPTRAY", {-30, -30, 0, 90, 120, -60, -15}},
      {"GRIPPING", {30, 0, 90, 100, 75, -15, 105}},
  };

  std::atomic_bool _halt_requested;
};

}  // namespace ek_challenger