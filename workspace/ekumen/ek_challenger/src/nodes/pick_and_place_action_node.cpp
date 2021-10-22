/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
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
  if (!getInput<std::string>("object_id", object_id)) {
    throw BT::RuntimeError("missing required input [object_id]");
  }

  geometry_msgs::PoseStamped target_pose;
  if (!getInput<geometry_msgs::PoseStamped>("target_pose", target_pose)) {
    throw BT::RuntimeError("missing required input [target_pose]");
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
    if (!plan_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  {
    ek_challenger::TaskConstructorExec srv;
    if (!exec_srv.call(srv)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void PickAndPlaceActionNode::halt() { halt_requested_ = true; }
