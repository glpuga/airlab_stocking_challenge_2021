/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third party
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// project
#include <ek_challenger/ScanShelves.h>

#include <ek_challenger/shelf_scanner.hpp>

namespace ek_challenger {

class ShelfScannerNode {
 public:
  ShelfScannerNode();

  bool run();

 private:
  ros::NodeHandle pnh_;

  sensor_msgs::Image depth_image_sample_;
  sensor_msgs::CameraInfo camera_info_sample_;

  ros::Publisher marker_pub_;

  ros::Subscriber depth_image_sub_;
  ros::Subscriber camera_info_sub_;

  ros::ServiceServer scan_shelf_srv_;

  bool scanShelvesCallback(ek_challenger::ScanShelves::Request &req,
                           ek_challenger::ScanShelves::Response &resp);

  bool waitForImage();

  void depthImageCallback(const sensor_msgs::Image &msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

  void publishMarkerMessage(const std::vector<geometry_msgs::PoseStamped>
                                &tomato_can_stocking_target_poses) const;
};

}  // namespace ek_challenger
