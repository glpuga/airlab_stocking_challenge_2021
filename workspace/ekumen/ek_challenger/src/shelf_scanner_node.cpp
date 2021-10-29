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
#include <ek_challenger/ScanShelves.h>

#include <ek_challenger/shelf_scanner_node.hpp>
#include <ek_challenger/tomato_can_dimensions.hpp>

namespace ek_challenger {

ShelfScannerNode::ShelfScannerNode() : pnh_{"~"} {
  marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("makers", 10);

  depth_image_sub_ =
      pnh_.subscribe("/xtion/depth_registered/image_raw", 1,
                     &ShelfScannerNode::depthImageCallback, this);
  camera_info_sub_ =
      pnh_.subscribe("/xtion/depth_registered/camera_info", 1,
                     &ShelfScannerNode::cameraInfoCallback, this);

  scan_shelf_srv_ =
      pnh_.advertiseService("find_tomato_can_loci_in_shelf",
                            &ShelfScannerNode::scanShelvesCallback, this);

  ROS_INFO_STREAM("Node started");
}

bool ShelfScannerNode::run() {
  ros::spin();
  return true;
}

bool ShelfScannerNode::scanShelvesCallback(
    ek_challenger::ScanShelves::Request &req,
    ek_challenger::ScanShelves::Response &resp) {
  TrayFinder tray_finder{req.volume_width, req.volume_height, req.volume_depth};

  ROS_INFO_STREAM("ScanShelves command received for volumen ("
                  << req.volume_width << " x " << req.volume_height << " x "
                  << req.volume_depth << ")");

  ROS_INFO_STREAM("Waiting for image campling...");

  auto tomato_cans_emtpy_loci = tray_finder.findFreeTomatoCanPoses(
      depth_image_sample_, camera_info_sample_);

  ROS_INFO_STREAM("Found " << tomato_cans_emtpy_loci.size() << " empty loci");

  publishMarkerMessage(tomato_cans_emtpy_loci);

  resp.tomato_cans_emtpy_loci = tomato_cans_emtpy_loci;
  return true;
}

void ShelfScannerNode::depthImageCallback(const sensor_msgs::Image &msg) {
  depth_image_sample_ = msg;
}

void ShelfScannerNode::cameraInfoCallback(const sensor_msgs::CameraInfo &msg) {
  camera_info_sample_ = msg;
}

void ShelfScannerNode::publishMarkerMessage(
    const std::vector<geometry_msgs::PoseStamped> &tomato_cans_emtpy_loci)
    const {
  if (!tomato_cans_emtpy_loci.size()) {
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = tomato_cans_emtpy_loci[0].header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "shelves_scanner_results";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = TomatoCanDimensions::width() * 1.0;
  marker.scale.y = TomatoCanDimensions::height() * 1.0;
  marker.scale.z = TomatoCanDimensions::depth() * 1.0;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (const auto &locus : tomato_cans_emtpy_loci) {
    geometry_msgs::Point p;
    p.x = locus.pose.position.x;
    p.y = locus.pose.position.y;
    p.z = locus.pose.position.z;
    marker.points.push_back(p);
  }

  marker_pub_.publish(marker);
}

}  // namespace ek_challenger
