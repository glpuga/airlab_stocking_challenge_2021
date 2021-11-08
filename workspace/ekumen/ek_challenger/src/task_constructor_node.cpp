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
#include <ros/ros.h>

// project
#include <ek_challenger/task_constructor_node.hpp>

namespace ek_challenger {

TaskConstructorNode::TaskConstructorNode() : nh_{} {
  plan_srv_ = nh_.advertiseService("task_constructor_plan",
                                   &TaskConstructorNode::planCallback, this);
  exec_srv_ = nh_.advertiseService("task_constructor_exec",
                                   &TaskConstructorNode::execCallback, this);
}

bool TaskConstructorNode::run() {
  ros::spin();
  return true;
}

bool TaskConstructorNode::planCallback(
    ek_challenger::TaskConstructorPlan::Request &req,
    ek_challenger::TaskConstructorPlan::Response &) {
  task_ptr_ = std::make_unique<ek_challenger::PickPlaceTask>(req.robot_side);

  if (!task_ptr_->build(req.source_object_id, req.target_pose, true,
                        req.flat_hand_mode)) {
    ROS_ERROR("Failed to build a plan description for Task Constructor");
    return false;
  }

  if (!task_ptr_->plan()) {
    ROS_ERROR("Failed to plan using Task Constructor");
    return false;
  }
  return true;
}

bool TaskConstructorNode::execCallback(
    ek_challenger::TaskConstructorExec::Request &,
    ek_challenger::TaskConstructorExec::Response &) {
  if (!task_ptr_) {
    ROS_ERROR("No preexisting plan, out of order call to exec");
    return false;
  }

  if (!task_ptr_->execute()) {
    ROS_ERROR("Failed to execute Task Constructor plan");
    return false;
  }

  task_ptr_.reset();

  return true;
}

}  // namespace ek_challenger
