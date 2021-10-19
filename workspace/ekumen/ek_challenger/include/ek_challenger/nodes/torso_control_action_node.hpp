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

  class TorsoControlActionNode : public BT::AsyncActionNode
  {
  public:
    TorsoControlActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("relative_height"),
      };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    const std::string torso_move_group_name_{"torso"};
    const double torso_max_range_{0.33};
    const double torso_min_range_{0.02};


    std::atomic_bool _halt_requested;
  };

}