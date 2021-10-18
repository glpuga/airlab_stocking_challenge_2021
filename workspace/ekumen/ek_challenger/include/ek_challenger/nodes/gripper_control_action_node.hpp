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
    MoveBaseActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("move_group"),
          BT::InputPort<double>("apperture"),
      };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
  };

}