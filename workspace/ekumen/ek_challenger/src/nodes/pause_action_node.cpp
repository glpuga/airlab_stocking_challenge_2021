/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/console.h>

// project
#include <ek_challenger/nodes/pause_action_node.hpp>

BT::NodeStatus PauseActionNode::tick() {
  int32_t time_limit_sec;
  if (!getInput<int32_t>("time_limit_sec", time_limit_sec)) {
    throw BT::RuntimeError("missing required input [time_limit_sec]");
  }

  ROS_INFO_STREAM("Sleeping for " << time_limit_sec << " seconds");

  const auto start_time = ros::Time::now();
  auto elapsed_time = [start_time]() { return ros::Time::now() - start_time; };

  while (!halt_requested_ && (elapsed_time() < ros::Duration(time_limit_sec))) {
    std::this_thread::sleep_for(time_step_);
  }

  ROS_INFO("Waking up from sleep");
  return BT::NodeStatus::SUCCESS;
}

void PauseActionNode::halt() { halt_requested_ = true; }
