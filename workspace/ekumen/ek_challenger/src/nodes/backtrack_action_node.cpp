/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <cmath>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>

// project
#include <ek_challenger/nodes/backtrack_action_node.hpp>

BT::NodeStatus BacktrackActionNode::tick() {
  double target_distance_m;

  if (!getInput<double>("target_distance_m", target_distance_m)) {
    throw BT::RuntimeError("missing required input [target_distance_m]");
  }

  ROS_INFO_STREAM("Backtracking...");

  ros::NodeHandle nh_{"~"};

  ros::Publisher pub =
      nh_.advertise<geometry_msgs::Twist>("/servoing_cmd_vel", 10);
  ros::Subscriber sub = nh_.subscribe("/mobile_base_controller/odom", 10,
                                      &BacktrackActionNode::odomCallback, this);

  const double speed = 0.2 * (target_distance_m < 0 ? -1 : 1);

  geometry_msgs::Twist msg;
  msg.linear.x = -speed;
  msg.linear.y = 0;
  msg.linear.z = 0;

  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;

  ros::Rate rate(20);

  double current_distance{0.0};

  known_start_pose_ = false;

  while (!halt_requested_ && (current_distance < target_distance_m)) {
    {
      std::lock_guard<std::mutex> lock{mutex_};
      if (known_start_pose_) {
        const auto xdiff = start_pose_.position.x - current_pose_.position.x;
        const auto ydiff = start_pose_.position.y - current_pose_.position.y;
        current_distance = std::sqrt(xdiff * xdiff + ydiff * ydiff);
      }
    }

    ROS_INFO_STREAM("Moved " << current_distance << " meters so far");

    pub.publish(msg);
    rate.sleep();
  }

  msg.linear.x = 0;
  pub.publish(msg);

  if (current_distance < target_distance_m) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void BacktrackActionNode::odomCallback(const nav_msgs::Odometry &msg) {
  std::lock_guard<std::mutex> lock{mutex_};
  if (!known_start_pose_) {
    known_start_pose_ = true;
    start_pose_ = msg.pose.pose;
  }
  current_pose_ = msg.pose.pose;
}

void BacktrackActionNode::halt() { halt_requested_ = true; }
