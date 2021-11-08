/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <fmt/format.h>
#include <std_srvs/Empty.h>

class ClearOctomapActionNode : public BT::AsyncActionNode {
 public:
  ClearOctomapActionNode(const std::string &name,
                         const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("side"),
    };
  }

  BT::NodeStatus tick() override {
    std::string side;
    if (!getInput<std::string>("side", side)) {
      throw BT::RuntimeError("missing required input [side]");
    }

    ros::ServiceClient clear_octomap_srv =
        ros::NodeHandle{}.serviceClient<std_srvs::Empty>(
            fmt::format("/{}/clear_octomap", side));

    clear_octomap_srv.waitForExistence();

    std_srvs::Empty srv;
    if (!clear_octomap_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}
};
