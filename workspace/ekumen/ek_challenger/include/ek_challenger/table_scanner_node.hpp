/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once
// standard library
#include <mutex>
#include <string>
#include <vector>

// third party
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// project
#include <ek_challenger/ScanTable.h>

#include <ek_challenger/ros_frame_transformer.hpp>
#include <ek_challenger/table_scanner.hpp>

namespace ek_challenger {

class TableScannerNode {
 public:
  TableScannerNode();

  bool run();

 private:
  const double min_interval_between_images_{1.0};

  ros::NodeHandle pnh_;

  ROSFrameTransformer tf_;

  std::mutex mutex_;

  sensor_msgs::Image depth_image_sample_;
  sensor_msgs::CameraInfo camera_info_msg_;

  ros::Publisher marker_pub_;
  ros::Publisher detections_pub_;

  ros::Subscriber depth_image_sub_;
  ros::Subscriber camera_info_sub_;

  ros::ServiceServer scan_table_srv_;

  bool valid_camera_info_{false};

  std::unique_ptr<TableScanner> table_scanner_ptr_;

  ros::Time latest_depth_image_timestamp_{ros::Time::now()};

  bool scanTableCallback(ek_challenger::ScanTable::Request &req,
                         ek_challenger::ScanTable::Response &resp);

  void depthImageCallback(const sensor_msgs::Image &msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

  void publishMarkerMessage(
      const std::vector<geometry_msgs::PoseStamped> &tomato_cans_loci) const;
};

}  // namespace ek_challenger
