/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// third party
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// project
#include <ek_challenger/ScanTable.h>

#include <ek_challenger/table_scanner_node.hpp>
#include <ek_challenger/tomato_can_dimensions.hpp>

namespace ek_challenger {

TableScannerNode::TableScannerNode() : pnh_{"~"} {
  marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("table_scan", 1);

  detections_pub_ = pnh_.advertise<sensor_msgs::Image>("detections", 1);

  depth_image_sub_ =
      pnh_.subscribe("/xtion/depth_registered/image_raw", 1,
                     &TableScannerNode::depthImageCallback, this);
  camera_info_sub_ =
      pnh_.subscribe("/xtion/depth_registered/camera_info", 1,
                     &TableScannerNode::cameraInfoCallback, this);

  scan_table_srv_ = pnh_.advertiseService(
      "scan_table", &TableScannerNode::scanTableCallback, this);

  ROS_INFO_STREAM("Node started");
}

bool TableScannerNode::run() {
  ros::spin();
  return true;
}

bool TableScannerNode::scanTableCallback(
    ek_challenger::ScanTable::Request &req,
    ek_challenger::ScanTable::Response &resp) {
  std::lock_guard<std::mutex> lock{mutex_};
  if (req.command == req.START_SCANNING) {
    // remove the previous markers
    publishMarkerMessage({});
    // create an instance of the scanner
    table_scanner_ptr_ = std::make_unique<TableScanner>(
        req.table_distance, req.table_width, req.table_depth);
    return true;
  } else if (req.command == req.PROCESS_RESULTS) {
    if (!table_scanner_ptr_) {
      return false;
    }

    sensor_msgs::Image detections;
    auto detected_tomato_cans = table_scanner_ptr_->processResult(detections);

    detections_pub_.publish(detections);

    resp.detected_tomato_cans = detected_tomato_cans;
    publishMarkerMessage(detected_tomato_cans);

    // delete the instance of the scanner
    table_scanner_ptr_.reset();

    return true;
  }

  // default failure case if the command is wrong
  return false;
}

void TableScannerNode::depthImageCallback(
    const sensor_msgs::Image &depth_image_msg) {
  std::lock_guard<std::mutex> lock{mutex_};

  if (table_scanner_ptr_ && valid_camera_info_) {
    // limit the max rate at which we process images
    auto time_since_latest_sample =
        depth_image_msg.header.stamp - latest_depth_image_timestamp_;
    if (time_since_latest_sample.toSec() < min_interval_between_images_) {
      // skip this frame
      return;
    }

    // process the frame
    latest_depth_image_timestamp_ = depth_image_msg.header.stamp;
    table_scanner_ptr_->processImage(depth_image_msg, camera_info_msg_, tf_);
  }
}

void TableScannerNode::cameraInfoCallback(const sensor_msgs::CameraInfo &msg) {
  std::lock_guard<std::mutex> lock{mutex_};
  camera_info_msg_ = msg;
  valid_camera_info_ = true;
}

void TableScannerNode::publishMarkerMessage(
    const std::vector<geometry_msgs::PoseStamped> &detected_tomato_cans) const {
  if (!detected_tomato_cans.size()) {
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = detected_tomato_cans[0].header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "shelves_scanner_results";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.a = 0.4;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  for (const auto &locus : detected_tomato_cans) {
    geometry_msgs::Point p;
    p.x = locus.pose.position.x;
    p.y = locus.pose.position.y;
    p.z = locus.pose.position.z;
    marker.points.push_back(p);
  }

  marker_pub_.publish(marker);
}

}  // namespace ek_challenger
