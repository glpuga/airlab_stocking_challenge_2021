/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/ScanShelves.h>

class ClearShelfObstaclesActionNode : public BT::AsyncActionNode {
 public:
  ClearShelfObstaclesActionNode(const std::string &name,
                                const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ros::ServiceClient scan_shelves_srv =
        ros::NodeHandle{}.serviceClient<ek_challenger::ScanShelves>(
            "/shelf_scanner_node/scan_shelves");

    scan_shelves_srv.waitForExistence();

    ek_challenger::ScanShelves srv;
    srv.request.command = srv.request.CLEAR;

    if (!scan_shelves_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {}
};
