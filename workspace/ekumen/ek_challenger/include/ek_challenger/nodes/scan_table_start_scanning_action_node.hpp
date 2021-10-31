/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/ScanTable.h>

class ScanTableStartScanningActionNode : public BT::AsyncActionNode {
 public:
  ScanTableStartScanningActionNode(const std::string &name,
                                   const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ros::ServiceClient scan_table_srv =
        ros::NodeHandle{}.serviceClient<ek_challenger::ScanTable>(
            "/table_scanner_node/scan_table");

    scan_table_srv.waitForExistence();

    {
      ek_challenger::ScanTable srv;
      srv.request.table_distance = 0.5;
      srv.request.table_width = 1.0;
      srv.request.table_depth = 0.6;

      srv.request.command = srv.request.START_SCANNING;

      if (!scan_table_srv.call(srv)) {
        return BT::NodeStatus::FAILURE;
      }

      return BT::NodeStatus::SUCCESS;
    }
  }

  void halt() override {}
};
