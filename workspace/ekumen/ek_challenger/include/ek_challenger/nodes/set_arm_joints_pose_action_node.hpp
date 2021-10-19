/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/nodes/set_arm_joints_pose_action_node.hpp>
#include <ek_challenger/arm_joints_pose.hpp>

namespace ek_challenger
{

  class SetArmsJointPoseActionNode : public BT::AsyncActionNode
  {
  public:
    SetArmsJointPoseActionNode(const std::string &name, const BT::NodeConfiguration &config, const std::string &move_group)
        : AsyncActionNode(name, config), move_group_{move_group}
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<ArmJointsPose>("joints_pose"),
      };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::string move_group_;

    std::atomic_bool _halt_requested;
  };

}