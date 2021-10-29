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

class TrayPickOccupiedLocusActionNode : public BT::AsyncActionNode {
 public:
  TrayPickOccupiedLocusActionNode(const std::string &name,
                                  const BT::NodeConfiguration &config,
                                  const ek_challenger::TrayData &trays)
      : AsyncActionNode(name, config), trays_{trays} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("tray_name"),
        BT::InputPort<std::string>("side"),
        BT::OutputPort<std::string>("locus_id"),
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
    auto success = trays_[tray_name]->pickOccupiedLocus(side, locus_id);

    if (!success) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput<std::string>("locus_id", locus_id);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}

 private:
  ek_challenger::TrayData trays_;
};
