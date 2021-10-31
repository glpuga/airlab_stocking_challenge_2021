/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/PoseStamped.h>

// project
#include <ek_challenger/tray_data.hpp>

class TraySetLociActionNode : public BT::AsyncActionNode {
 public:
  TraySetLociActionNode(const std::string &name,
                        const BT::NodeConfiguration &config,
                        const ek_challenger::TrayData &trays)
      : AsyncActionNode(name, config), trays_{trays} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("tray_name"),
        BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("locus_poses"),
        BT::InputPort<bool>("occupied")};
  }

  BT::NodeStatus tick() override {
    std::string tray_name;
    if (!getInput<std::string>("tray_name", tray_name)) {
      throw BT::RuntimeError("missing required input [tray_name]");
    }

    std::vector<geometry_msgs::PoseStamped> locus_poses;
    if (!getInput<std::vector<geometry_msgs::PoseStamped>>("locus_poses",
                                                           locus_poses)) {
      throw BT::RuntimeError("missing required input [locus_poses]");
    }

    bool occupied;
    if (!getInput<bool>("occupied", occupied)) {
      throw BT::RuntimeError("missing required input [occupied]");
    }

    trays_[tray_name]->clear();

    auto insert_in_tray = [&, this](const geometry_msgs::PoseStamped &pose) {
      trays_[tray_name]->addLocus(pose, occupied, 0);
    };
    std::for_each(locus_poses.begin(), locus_poses.end(), insert_in_tray);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}

 private:
  ek_challenger::TrayData trays_;
};
