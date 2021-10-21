/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <atomic>
#include <mutex>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class BacktrackActionNode : public BT::AsyncActionNode
{
public:
  BacktrackActionNode(const std::string &name, const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<int32_t>("target_distance_m"), /* Distance to backtrack, in meters */
    };
  }

  BT::NodeStatus tick() override;

  void halt() override;

private:
  std::mutex mutex_;
  bool known_start_pose_{false};
  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose current_pose_;

  std::atomic_bool halt_requested_{false};

  void odomCallback(const nav_msgs::Odometry &msg);
};
