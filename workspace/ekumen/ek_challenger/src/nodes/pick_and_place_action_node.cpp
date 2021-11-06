/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <mutex>
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

// project
#include <ek_challenger/TaskConstructorExec.h>
#include <ek_challenger/TaskConstructorPlan.h>

#include <ek_challenger/nodes/pick_and_place_action_node.hpp>

BT::NodeStatus PickAndPlaceActionNode::tick() {
  std::string object_id;

  static std::mutex busy_mutex;

  if (!getInput<std::string>("object_id", object_id)) {
    throw BT::RuntimeError("missing required input [object_id]");
  }

  geometry_msgs::PoseStamped target_pose;
  if (!getInput<geometry_msgs::PoseStamped>("target_pose", target_pose)) {
    throw BT::RuntimeError("missing required input [target_pose]");
  }

  bool flat_hand_mode{false};
  if (!getInput<bool>("flat_hand_mode", flat_hand_mode)) {
    throw BT::RuntimeError("missing required input [flat_hand_mode]");
  }

  ros::NodeHandle nh;
  ros::ServiceClient plan_srv =
      nh.serviceClient<ek_challenger::TaskConstructorPlan>(
          "/" + side_ + "/task_constructor_plan");

  ros::ServiceClient exec_srv =
      nh.serviceClient<ek_challenger::TaskConstructorExec>(
          "/" + side_ + "/task_constructor_exec");

  plan_srv.waitForExistence();
  exec_srv.waitForExistence();

  {
    ek_challenger::TaskConstructorPlan srv;
    srv.request.robot_side = side_;
    srv.request.source_object_id = object_id;
    srv.request.target_pose = target_pose;
    srv.request.flat_hand_mode = flat_hand_mode;
    if (!plan_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  {
    // only one arm at a time can be here
    std::lock_guard<std::mutex> l{busy_mutex};

    ek_challenger::TaskConstructorExec srv;
    if (!exec_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void PickAndPlaceActionNode::halt() { halt_requested_ = true; }
