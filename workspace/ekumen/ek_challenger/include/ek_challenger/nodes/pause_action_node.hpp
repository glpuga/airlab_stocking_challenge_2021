/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <atomic>
#include <chrono>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

/** @brief Node that stops execution for a programmable amount of seconds to generate a pause
 */
class PauseActionNode : public BT::AsyncActionNode
{
public:
  PauseActionNode(const std::string &name, const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<int32_t>("time_limit_sec"), /* Amount of time in seconds to sleep */
    };
  }

  BT::NodeStatus tick() override;

  void halt() override;

private:
  std::chrono::milliseconds time_step_{20};

  std::atomic_bool halt_requested_{false};
};
