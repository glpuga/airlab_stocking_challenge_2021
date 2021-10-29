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

class TrayUpdatePlanningSceneActionNode : public BT::AsyncActionNode {
 public:
  TrayUpdatePlanningSceneActionNode(const std::string &name,
                                    const BT::NodeConfiguration &config,
                                    const ek_challenger::TrayData &trays)
      : AsyncActionNode(name, config), trays_{trays} {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    for (const auto &tray_data : trays_) {
      tray_data.second->updatePlanningScene();
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override{};

 private:
  ek_challenger::TrayData trays_;
};
