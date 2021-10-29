/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/tray_data.hpp>
#include <ek_challenger/tray_model_interface.hpp>

class TrayPickEmptyLocusActionNode : public BT::AsyncActionNode {
 public:
  TrayPickEmptyLocusActionNode(const std::string &name,
                               const BT::NodeConfiguration &config,
                               const ek_challenger::TrayData &trays)
      : AsyncActionNode(name, config), trays_{trays} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("tray_name"),
        BT::InputPort<std::string>("side"),
        BT::OutputPort<std::string>("locus_id"),
        BT::OutputPort<geometry_msgs::PoseStamped>("locus_pose"),
    };
  }

  BT::NodeStatus tick() override {
    std::string tray_name;
    if (!getInput<std::string>("tray_name", tray_name)) {
      throw BT::RuntimeError("missing required input [tray_name]");
    }

    std::string side_str;
    if (!getInput<std::string>("side", side_str)) {
      throw BT::RuntimeError("missing required input [side]");
    }

    ek_challenger::TrayModelInterface::Side side;

    if (side_str == "LEFT") {
      side = ek_challenger::TrayModelInterface::Side::LEFT;
    } else if (side_str == "RIGHT") {
      side = ek_challenger::TrayModelInterface::Side::RIGHT;
    } else {
      throw std::runtime_error("Bad side value");
    }

    std::string locus_id;
    auto success = trays_[tray_name]->pickEmptyLocus(side, locus_id);

    if (!success) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PoseStamped locus_pose =
        trays_[tray_name]->getLocusPose(locus_id);

    setOutput<std::string>("locus_id", locus_id);
    setOutput<geometry_msgs::PoseStamped>("locus_pose", locus_pose);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}

 private:
  ek_challenger::TrayData trays_;
};
