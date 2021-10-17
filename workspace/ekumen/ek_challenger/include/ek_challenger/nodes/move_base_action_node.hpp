/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/Pose2D.h>

// project
#include <ek_challenger/data_conversions.hpp>

namespace ek_challenger
{

  class MoveBaseActionNode : public BT::AsyncActionNode
  {
  public:
    MoveBaseActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<geometry_msgs::Pose2D>("goal")};
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
  };

}