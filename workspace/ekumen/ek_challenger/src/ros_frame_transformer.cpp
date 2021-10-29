/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// geometry_msgs::PoseStamped
#include <ek_challenger/ros_frame_transformer.hpp>

namespace ek_challenger {

ROSFrameTransformer::ROSFrameTransformer() {}

geometry_msgs::PoseStamped ROSFrameTransformer::transformPoseToFrame(
    const geometry_msgs::PoseStamped &pose,
    const std::string &new_frame_id) const {
  geometry_msgs::PoseStamped output_pose;

  try {
    auto start = ros::Time::now();
    tf2::doTransform(
        pose, output_pose,
        tf_buffer_.lookupTransform(new_frame_id, pose.header.frame_id,
                                   ros::Time(0), tf_timeout_));
    auto end = ros::Time::now();
    if (end - start > ros::Duration(0.1)) {
      ROS_WARN_STREAM("A tf2 transform took " << (end - start).toSec()
                                              << " seconds to complete!");
    }
  } catch (const std::exception &ex) {
    ROS_ERROR_STREAM("Error while tranforming between frames "
                     << pose.header.frame_id << " and " << new_frame_id << ": "
                     << ex.what());
    throw std::runtime_error{"Failed ROS frame transformation"};
  }

  return output_pose;
}

}  // namespace ek_challenger
