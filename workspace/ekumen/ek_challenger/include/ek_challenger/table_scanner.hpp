/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>
#include <tuple>
#include <vector>

// third party
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// project
#include <ek_challenger/ros_frame_transformer.hpp>
#include <ek_challenger/tray_finder.hpp>

namespace ek_challenger {

class TableScanner {
 public:
  TableScanner(const double table_distance_to_base, const double table_width,
               const double table_depth);

  void processImage(const sensor_msgs::Image &image,
                    const sensor_msgs::CameraInfo &camera_info,
                    const ROSFrameTransformer &tf);

  std::vector<geometry_msgs::PoseStamped> processResult(
      sensor_msgs::Image &proc_image) const;

 private:
  const std::string base_link_frame_{"base_link"};

  // buffer granularity
  const double buffer_delta_{0.005};

  const double table_distance_to_base_;
  const double table_width_;
  const double table_depth_;

  const int32_t buffer_width_;
  const int32_t buffer_height_;

  std::vector<double> buffer_;

  std::pair<int32_t, int32_t> convertXYtoUV(const double x,
                                            const double y) const;

  int32_t uvToLinearIndex(const int32_t u, const int32_t v) const;
};

}  // namespace ek_challenger
