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

class TraySetLocusStateActionNode : public BT::AsyncActionNode {
 public:
  TraySetLocusStateActionNode(const std::string &name,
                              const BT::NodeConfiguration &config,
                              const ek_challenger::TrayData &trays)
      : AsyncActionNode(name, config), trays_{trays} {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("tray_name"),
            BT::InputPort<std::string>("locus_id"),
            BT::InputPort<bool>("occupied")};
  }

  BT::NodeStatus tick() override {
    std::string tray_name;
    if (!getInput<std::string>("tray_name", tray_name)) {
      throw BT::RuntimeError("missing required input [tray_name]");
    }

    std::string locus_id;
    if (!getInput<std::string>("locus_id", locus_id)) {
      throw BT::RuntimeError("missing required input [locus_id]");
    }

    bool occupied;
    if (!getInput<bool>("occupied", occupied)) {
      throw BT::RuntimeError("missing required input [occupied]");
    }

    trays_[tray_name]->setLocusState(locus_id, occupied);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}

 private:
  ek_challenger::TrayData trays_;
};
