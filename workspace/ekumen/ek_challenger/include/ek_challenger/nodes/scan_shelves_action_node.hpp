/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/ScanShelves.h>

class ScanShelvesActionNode : public BT::AsyncActionNode {
 public:
  ScanShelvesActionNode(const std::string &name,
                        const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>(
            "tomato_can_stocking_target_poses"),
    };
  }

  BT::NodeStatus tick() override {
    ros::ServiceClient scan_shelves_srv =
        ros::NodeHandle{}.serviceClient<ek_challenger::ScanShelves>(
            "/shelf_scanner_node/scan_shelves");

    scan_shelves_srv.waitForExistence();

    {
      ek_challenger::ScanShelves srv;
      srv.request.volume_width = 0.8;
      srv.request.volume_height = 1.2;
      srv.request.volume_depth = 2.2;

      if (!scan_shelves_srv.call(srv)) {
        return BT::NodeStatus::FAILURE;
      }

      if (srv.response.tomato_can_stocking_target_poses.size() < 1) {
        return BT::NodeStatus::FAILURE;
      }

      setOutput<std::vector<geometry_msgs::PoseStamped>>(
          "tomato_can_stocking_target_poses",
          srv.response.tomato_can_stocking_target_poses);

      return BT::NodeStatus::SUCCESS;
    }
  }

  void halt() override {}
};
