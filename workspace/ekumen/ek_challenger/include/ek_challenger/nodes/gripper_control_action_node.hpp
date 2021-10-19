/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

namespace ek_challenger
{

  class GripperControlActionNode : public BT::AsyncActionNode
  {
  public:
    GripperControlActionNode(const std::string &name, const BT::NodeConfiguration &config, const std::string &move_group)
        : AsyncActionNode(name, config), move_group_{move_group}
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("relative_gripper_gap"),
      };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    const double max_gripper_gap_{0.041};

    std::string move_group_;

    std::atomic_bool _halt_requested;
  };

}