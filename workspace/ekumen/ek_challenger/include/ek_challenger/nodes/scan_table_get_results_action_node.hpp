/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/PoseStamped.h>

// project
#include <ek_challenger/ScanTable.h>

class ScanTableGetResultsActionNode : public BT::AsyncActionNode {
 public:
  ScanTableGetResultsActionNode(const std::string &name,
                                const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>(
            "detected_tomato_cans"),
    };
  }

  BT::NodeStatus tick() override {
    ros::ServiceClient scan_table_srv =
        ros::NodeHandle{}.serviceClient<ek_challenger::ScanTable>(
            "/table_scanner_node/scan_table");

    scan_table_srv.waitForExistence();

    {
      ek_challenger::ScanTable srv;
      srv.request.command = srv.request.PROCESS_RESULTS;
      if (!scan_table_srv.call(srv)) {
        return BT::NodeStatus::FAILURE;
      }

      setOutput<std::vector<geometry_msgs::PoseStamped>>(
          "detected_tomato_cans", srv.response.detected_tomato_cans);

      return BT::NodeStatus::SUCCESS;
    }
  }

  void halt() override {}
};
