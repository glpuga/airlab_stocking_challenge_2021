/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard libraries
#include <cstdint>
#include <vector>

// third party
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// project
#include <ek_challenger/tomato_can_dimensions.hpp>

namespace ek_challenger {

class ShelfScanner {
 public:
  ShelfScanner(const double width, const double height, const double depth);

  std::vector<geometry_msgs::PoseStamped> scanShelvesForStockingTargetPoses(
      const sensor_msgs::Image &depth_image,
      const sensor_msgs::CameraInfo &camera_info);

 private:
  enum class CellContent {
    Unknown,
    Empty,
    DoNotUse,
    Obstacle,
    TomatoCan,
  };

  struct BufferIndex {
    int32_t u;
    int32_t v;
    int32_t w;
  };

  geometry_msgs::Pose origin_pose_;

  const int32_t cans_width_;
  const int32_t cans_height_;
  const int32_t cans_depth_;

  const double volumen_width_;
  const double volumen_height_;
  const double volumen_depth_;

  const int32_t slice_stride_;

  std::vector<CellContent> buffer_;

  bool isWithinBuffer(const double x, const double y, const double z) const;

  bool isWithinRange(const double a, const double x, const double b) const;

  BufferIndex pointToBufferIndex(const double x, const double y,
                                 const double z) const;

  geometry_msgs::Pose bufferIndexToCenterPose(
      const BufferIndex &buffer_index) const;

  int32_t bufferIndexToLinearIndex(const BufferIndex &index) const;

  BufferIndex linearIndexToBufferIndex(const int32_t &linear_index) const;

  bool linearIndexIsWithinBuffer(const int32_t &linear_index) const;

  int32_t linearIndexOfElementBelow(const int32_t &linear_index) const;

  int32_t linearIndexOfElementAbove(const int32_t &linear_index) const;

  int32_t linearIndexOfElementInFront(const int32_t &linear_index) const;

  void addDepthInformation(const sensor_msgs::Image &depth_image,
                           const sensor_msgs::CameraInfo &camera_info,
                           std::vector<CellContent> &buffer,
                           const double y_offset) const;
};

}  // namespace ek_challenger
