/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

// third party
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc.hpp>

// project
#include <ek_challenger/ros_frame_transformer.hpp>
#include <ek_challenger/table_scanner.hpp>
#include <ek_challenger/tomato_can_dimensions.hpp>
namespace ek_challenger {

TableScanner::TableScanner(const double table_distance_to_base,
                           const double table_width, const double table_depth)
    : table_distance_to_base_{table_distance_to_base},
      table_width_{table_width},
      table_depth_{table_depth},
      buffer_width_{static_cast<int32_t>(table_depth_ / buffer_delta_) + 1},
      buffer_height_{static_cast<int32_t>(table_width_ / buffer_delta_) + 1} {
  // initialize the buffer_
  buffer_.resize(buffer_width_ * buffer_height_);
  std::for_each(buffer_.begin(), buffer_.end(),
                [](double &value) { value = 0.0; });
}

void TableScanner::processImage(const sensor_msgs::Image &depth_image,
                                const sensor_msgs::CameraInfo &camera_info,
                                const ROSFrameTransformer &tf) {
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);

  auto cv_image = cv_bridge::toCvCopy(depth_image);

  geometry_msgs::PoseStamped pixel_pose_stamped;
  pixel_pose_stamped.header = depth_image.header;
  pixel_pose_stamped.pose.orientation.w = 1.0;

  auto is_within_range = [](const double a, const double x, const double b) {
    return (a < x) && (x < b);
  };

  for (int32_t u = 0; u < cv_image->image.cols; ++u) {
    for (int32_t v = 0; v < cv_image->image.rows; ++v) {
      const auto depth = cv_image->image.at<float>(v, u);

      if (std::isnan(depth)) {
        continue;
      }

      cv::Point2d uv_coords;
      uv_coords.x = u;
      uv_coords.y = v;

      const auto vec = model.projectPixelTo3dRay(uv_coords);
      const auto xyz = depth * vec;

      pixel_pose_stamped.pose.position.x = xyz.x;
      pixel_pose_stamped.pose.position.y = xyz.y;
      pixel_pose_stamped.pose.position.z = xyz.z;

      if (std::isnan(xyz.x) && std::isnan(xyz.y) && std::isnan(xyz.z)) {
        continue;
      }

      // transform to base_link
      const auto pose_in_base =
          tf.transformPoseToFrame(pixel_pose_stamped, base_link_frame_);

      // relative to a frame aligned with base_link, but displaced to the right
      // to the corner of the area of interest
      const double x_relative_to_table =
          pose_in_base.pose.position.x - table_distance_to_base_;
      const double y_relative_to_table =
          pose_in_base.pose.position.y + table_width_ / 2.0;
      const double z_relative_to_base = pose_in_base.pose.position.z;

      // check if it is within the range of interest, store the z value if it is
      if (is_within_range(0.0, x_relative_to_table, table_depth_) &&
          is_within_range(0.0, y_relative_to_table, table_width_)) {
        const auto uv = convertXYtoUV(x_relative_to_table, y_relative_to_table);
        if (z_relative_to_base > 0) {
          buffer_[uvToLinearIndex(uv.first, uv.second)] = z_relative_to_base;
        }
      }
    }
  }
}

std::vector<geometry_msgs::PoseStamped> TableScanner::processResult(
    sensor_msgs::Image &detections_image) const {
  // create an opencv image for processing. Remember that opencv
  // uses rows/cols for order, not x/y
  cv::Mat image = cv::Mat::zeros(buffer_height_, buffer_width_, CV_8UC1);

  double z_max = 0.0;
  auto determine_max_z = [&z_max](const double &z) {
    z_max = z_max > z ? z_max : z;
  };
  std::for_each(buffer_.begin(), buffer_.end(), determine_max_z);

  ROS_INFO_STREAM("z_max value: " << z_max);

  // determine the local threshold, using the height of a tomato can as
  // reference
  const double z_threshold = z_max - TomatoCanDimensions::height() / 2.0;

  for (int32_t u = 0; u < buffer_width_; ++u) {
    for (int32_t v = 0; v < buffer_height_; ++v) {
      const auto color = buffer_[uvToLinearIndex(u, v)] > z_threshold ? 200 : 0;
      image.at<uchar>(v, u) = color;
    }
  }

  cv_bridge::CvImage cv_bridge_image(std_msgs::Header{}, "8UC1", image);
  cv_bridge_image.toImageMsg(detections_image);

  // soften the borders with and erode/
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
  cv::morphologyEx(image, image, cv::MORPH_OPEN, element);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  ROS_INFO_STREAM("Detected contours: " << contours.size());

  auto is_within_range = [](const double a, const double x, const double b) {
    return (a <= x) && (x <= b);
  };

  // determine minimum enclosing circles for the contours, filter them and
  // create list of tomato cans!
  std::vector<geometry_msgs::PoseStamped> detected_tomato_cans;

  for (size_t i = 0; i < contours.size(); i++) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours[i], center, radius);

    ROS_INFO_STREAM("Detection centetr and radious: c=" << center << " r=" << radius);

    const auto is_fully_within_buffer =
        is_within_range(radius, center.x, buffer_width_ - radius) &&
        is_within_range(radius, center.y, buffer_height_ - radius);

    const auto radius_is_correct = is_within_range(
        TomatoCanDimensions::width() * 0.7, buffer_delta_ * 2 * radius,
        TomatoCanDimensions::width() * 1.3);

    ROS_INFO_STREAM("Filters: " << is_fully_within_buffer << radius_is_correct);

    if (is_fully_within_buffer && radius_is_correct) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = base_link_frame_;
      pose.pose.position.x = center.x * buffer_delta_ + table_distance_to_base_;
      pose.pose.position.y = center.y * buffer_delta_ - table_width_ / 2.0;
      pose.pose.position.z = z_threshold;
      pose.pose.orientation.w = 1;

      detected_tomato_cans.push_back(pose);
    }
  }

  ROS_INFO_STREAM("Detection count after filtering: " << detected_tomato_cans.size());

  return detected_tomato_cans;
}

std::pair<int32_t, int32_t> TableScanner::convertXYtoUV(const double x,
                                                        const double y) const {
  const int32_t u = static_cast<int32_t>(std::round(x / buffer_delta_));
  const int32_t v = static_cast<int32_t>(std::round(y / buffer_delta_));
  return std::make_pair(u, v);
}

int32_t TableScanner::uvToLinearIndex(const int32_t u, const int32_t v) const {
  return v * buffer_width_ + u;
}

}  // namespace ek_challenger
