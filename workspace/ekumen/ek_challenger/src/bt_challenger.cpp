/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <string>

// third party
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/console.h>

// project
#include <ek_challenger/bt_challenger.hpp>
#include <ek_challenger/nodes/backtrack_action_node.hpp>
#include <ek_challenger/nodes/create_obstacle_action_node.hpp>
#include <ek_challenger/nodes/get_arm_joints_for_pose_action_node.hpp>
#include <ek_challenger/nodes/gripper_control_action_node.hpp>
#include <ek_challenger/nodes/head_control_action_node.hpp>
#include <ek_challenger/nodes/move_base_action_node.hpp>
#include <ek_challenger/nodes/pause_action_node.hpp>
#include <ek_challenger/nodes/pick_and_place_action_node.hpp>
#include <ek_challenger/nodes/set_arm_joints_pose_action_node.hpp>
#include <ek_challenger/nodes/torso_control_action_node.hpp>
#include <ek_challenger/nodes/tray_pick_empty_locus_action_node.hpp>
#include <ek_challenger/nodes/tray_pick_occupied_locus_action_node.hpp>
#include <ek_challenger/nodes/tray_release_locus_action_node.hpp>
#include <ek_challenger/nodes/tray_set_locus_state_action_node.hpp>
#include <ek_challenger/nodes/tray_update_planning_scene_action_node.hpp>
#include <ek_challenger/tray_data.hpp>
#include <ek_challenger/tray_model_table.hpp>

namespace ek_challenger {
BehaviorTreeNode::BehaviorTreeNode() : nh_{"~"} {
  BT::BehaviorTreeFactory factory;
  registerNodes(factory);

  std::string tree_description_filepath;
  if (!ros::param::get("~tree_description_filepath",
                       tree_description_filepath)) {
    throw std::runtime_error("The behavior tree filepath parameter is missing");
  }

  ROS_WARN_STREAM("Creating behavior tree from description file: "
                  << tree_description_filepath);
  tree_ = std::make_unique<BT::Tree>(
      factory.createTreeFromFile(tree_description_filepath));

  ROS_WARN_STREAM("Attaching CoutLogger to tree");
  cout_logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);
  ROS_WARN_STREAM("Attaching ZMQ Publisher to tree");
  zmq_logger_ = std::make_unique<BT::PublisherZMQ>(*tree_);
}

bool BehaviorTreeNode::run() {
  ros::Rate rate(tick_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    tree_->tickRoot();
  }
  return true;
}

void BehaviorTreeNode::registerNodes(BT::BehaviorTreeFactory &factory) {
  const std::string right_channel_ns_{"right"};
  const std::string left_channel_ns_{"left"};

  factory.registerNodeType<PauseActionNode>("Pause");
  factory.registerNodeType<HeadControlActionNode>("HeadControl");
  factory.registerNodeType<MoveBaseActionNode>("MoveBase");
  factory.registerNodeType<TorsoControlActionNode>("TorsoControl");
  factory.registerNodeType<BacktrackActionNode>("Backtrack");
  factory.registerNodeType<CreateObstacleActionNode>("CreateObstacle");

  factory.registerBuilder<PickAndPlaceActionNode>(
      "PickAndPlaceLeft",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<PickAndPlaceActionNode>(nm, cfg, "left",
                                                        left_channel_ns_);
      });

  factory.registerBuilder<PickAndPlaceActionNode>(
      "PickAndPlaceRight",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<PickAndPlaceActionNode>(nm, cfg, "right",
                                                        right_channel_ns_);
      });

  factory.registerBuilder<GripperControlActionNode>(
      "GripperControlRight",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<GripperControlActionNode>(
            nm, cfg, "gripper_right", right_channel_ns_);
      });

  factory.registerBuilder<GripperControlActionNode>(
      "GripperControlLeft",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<GripperControlActionNode>(
            nm, cfg, "gripper_left", left_channel_ns_);
      });

  factory.registerBuilder<SetArmsJointPoseActionNode>(
      "SetArmJointsRight",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<SetArmsJointPoseActionNode>(
            nm, cfg, "arm_right", right_channel_ns_);
      });

  factory.registerBuilder<SetArmsJointPoseActionNode>(
      "SetArmJointsLeft",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<SetArmsJointPoseActionNode>(nm, cfg, "arm_left",
                                                            left_channel_ns_);
      });

  factory.registerNodeType<GetArmJointsForPoseActionNode>(
      "GetArmJointsForPose");

  auto make_pose_stamped = [](const std::string &frame, const double x,
                              const double y, const double z) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1;
    return pose;
  };

  auto make_pose = [](const double x, const double y) {
    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    return pose;
  };

  TrayData trays;

  trays["table"] = std::make_shared<TrayModelTable>(
      make_pose_stamped("base_link", 0.6, 0.0, 0.70),
      std::vector<std::string>{"left", "right"});
  trays["table"]->addLocus(make_pose(0.1, 0.15), true, 0);
  trays["table"]->addLocus(make_pose(0.1, 0.05), true, 0);
  trays["table"]->addLocus(make_pose(0.1, -0.05), true, 0);
  trays["table"]->addLocus(make_pose(0.1, -0.15), true, 0);
  trays["table"]->addLocus(make_pose(-0.1, 0.15), true, 0);
  trays["table"]->addLocus(make_pose(-0.1, 0.05), true, 0);
  trays["table"]->addLocus(make_pose(-0.1, -0.05), true, 0);
  trays["table"]->addLocus(make_pose(-0.1, -0.15), true, 0);

  trays["backtray"] = std::make_shared<TrayModelTable>(
      make_pose_stamped("torso_lift_link", 0.0, 0.0, 0.01),
      std::vector<std::string>{"left", "right"});
  trays["backtray"]->addLocus(make_pose(0.08, -0.1), false, 0);
  trays["backtray"]->addLocus(make_pose(0.08, 0.1), false, 0);
  trays["backtray"]->addLocus(make_pose(-0.02, -0.1), false, 0);
  trays["backtray"]->addLocus(make_pose(-0.02, 0.1), false, 0);
  trays["backtray"]->addLocus(make_pose(-0.12, -0.1), false, 0);
  trays["backtray"]->addLocus(make_pose(-0.12, 0.1), false, 0);

  trays["shelf"] = std::make_shared<TrayModelTable>(
      make_pose_stamped("base_link", 0.5, 0.0, 1.00),
      std::vector<std::string>{"left", "right"});
  trays["table"]->addLocus(make_pose(0.1, 0.15), false, 0);
  trays["shelf"]->addLocus(make_pose(0.1, 0.05), false, 0);
  trays["shelf"]->addLocus(make_pose(0.1, -0.05), false, 0);
  trays["shelf"]->addLocus(make_pose(0.1, -0.15), false, 0);
  trays["shelf"]->addLocus(make_pose(-0.1, 0.15), false, 0);
  trays["shelf"]->addLocus(make_pose(-0.1, 0.05), false, 0);
  trays["shelf"]->addLocus(make_pose(-0.1, -0.05), false, 0);
  trays["shelf"]->addLocus(make_pose(-0.1, -0.15), false, 0);

  factory.registerBuilder<TrayUpdatePlanningSceneActionNode>(
      "TrayUpdatePlanningScene",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<TrayUpdatePlanningSceneActionNode>(nm, cfg,
                                                                   trays);
      });

  factory.registerBuilder<TrayPickEmptyLocusActionNode>(
      "TrayPickEmptyLocus",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<TrayPickEmptyLocusActionNode>(nm, cfg, trays);
      });

  factory.registerBuilder<TrayPickOccupiedLocusActionNode>(
      "TrayPickOccupiedLocus",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<TrayPickOccupiedLocusActionNode>(nm, cfg,
                                                                 trays);
      });

  factory.registerBuilder<TrayReleaseLocusActionNode>(
      "TrayReleaseLocus",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<TrayReleaseLocusActionNode>(nm, cfg, trays);
      });

  factory.registerBuilder<TraySetLocusStateActionNode>(
      "TraySetLocusState",
      [=](const std::string &nm, const BT::NodeConfiguration &cfg) {
        return std::make_unique<TraySetLocusStateActionNode>(nm, cfg, trays);
      });
}

}  // namespace ek_challenger
