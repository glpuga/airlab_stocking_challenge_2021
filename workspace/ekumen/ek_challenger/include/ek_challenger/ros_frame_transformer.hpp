/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// roscpp
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace ek_challenger {

class ROSFrameTransformer {
 public:
  ROSFrameTransformer();

  geometry_msgs::PoseStamped transformPoseToFrame(
      const geometry_msgs::PoseStamped &pose,
      const std::string &new_frame_id) const;

 private:
  const ros::Duration tf_timeout_{1.5};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

}  // namespace ek_challenger
