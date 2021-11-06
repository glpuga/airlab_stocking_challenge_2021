/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard libraries
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

// third party
#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// project 
#include <ek_challenger/shelf_scanner.hpp>
#include <ek_challenger/tomato_can_dimensions.hpp>

namespace ek_challenger {

namespace {
constexpr double can_slices = 8;
constexpr double minimum_slices_for_usable_space = 2 * can_slices;

constexpr double cell_width = TomatoCanDimensions::width() * 1.4;
constexpr double cell_depth = TomatoCanDimensions::depth() * 1.05;
constexpr double cell_height =
    TomatoCanDimensions::height() * 1.05 / static_cast<double>(can_slices);
};  // namespace

ShelfScanner::ShelfScanner(const double width, const double height,
                           const double depth)
    : cans_width_{static_cast<int32_t>(width / cell_width)},
      cans_height_{static_cast<int32_t>(height / cell_height)},
      cans_depth_{static_cast<int32_t>(depth / cell_depth)},
      volumen_width_{cans_width_ * cell_width},
      volumen_height_{cans_height_ * cell_height},
      volumen_depth_{cans_depth_ * cell_depth},
      slice_stride_{cans_width_ * cans_height_} {
  buffer_.resize(cans_width_ * cans_height_ * cans_depth_);

  origin_pose_.position.x = -volumen_width_ / 2.0;
  origin_pose_.position.y = -volumen_height_ / 2.0;
  origin_pose_.position.z = 0;
  origin_pose_.orientation.w = 1.0;
}

bool ShelfScanner::isWithinBuffer(const double x, const double y,
                                  const double z) const {
  return isWithinRange(origin_pose_.position.x, x,
                       origin_pose_.position.x + volumen_width_) &&
         isWithinRange(origin_pose_.position.y, y,
                       origin_pose_.position.y + volumen_height_) &&
         isWithinRange(origin_pose_.position.z, z,
                       origin_pose_.position.z + volumen_depth_);
}

bool ShelfScanner::isWithinRange(const double a, const double x,
                                 const double b) const {
  return (a <= x) && (x <= b);
}

ShelfScanner::BufferIndex ShelfScanner::pointToBufferIndex(
    const double x, const double y, const double z) const {
  BufferIndex index;
  index.u = static_cast<int32_t>(
      std::round(std::floor((x - origin_pose_.position.x) / cell_width)));
  index.v = static_cast<int32_t>(
      std::round(std::floor((y - origin_pose_.position.y) / cell_height)));
  index.w = static_cast<int32_t>(
      std::round(std::floor((z - origin_pose_.position.z) / cell_depth)));
  return index;
}

geometry_msgs::Pose ShelfScanner::bufferIndexToCenterPose(
    const BufferIndex &buffer_index) const {
  geometry_msgs::Pose pose;
  pose.position.x =
      buffer_index.u * cell_width + cell_width * 0.5 + origin_pose_.position.x;
  pose.position.y = buffer_index.v * cell_height + cell_height -
                    TomatoCanDimensions::height() * 0.5 +
                    origin_pose_.position.y;
  pose.position.z =
      buffer_index.w * cell_depth + cell_depth * 0.5 + origin_pose_.position.z;
  pose.orientation.w = 1.0;
  return pose;
}

int32_t ShelfScanner::bufferIndexToLinearIndex(const BufferIndex &index) const {
  return index.w * slice_stride_ + index.v * cans_width_ + index.u;
}

ShelfScanner::BufferIndex ShelfScanner::linearIndexToBufferIndex(
    const int32_t &linear_index) const {
  BufferIndex buffer_index;
  buffer_index.w = linear_index / slice_stride_;

  auto local_linear_index = linear_index - buffer_index.w * slice_stride_;
  buffer_index.v = local_linear_index / cans_width_;

  buffer_index.u = local_linear_index - buffer_index.v * cans_width_;

  return buffer_index;
}

bool ShelfScanner::linearIndexIsWithinBuffer(
    const int32_t &linear_index) const {
  return static_cast<size_t>(linear_index) < buffer_.size();
}

int32_t ShelfScanner::linearIndexOfElementBelow(
    const int32_t &linear_index) const {
  return linear_index + cans_width_;
}

int32_t ShelfScanner::linearIndexOfElementAbove(
    const int32_t &linear_index) const {
  return linear_index - cans_width_;
}

int32_t ShelfScanner::linearIndexOfElementInFront(
    const int32_t &linear_index) const {
  return linear_index - slice_stride_;
}

std::vector<geometry_msgs::PoseStamped>
ShelfScanner::scanShelvesForStockingTargetPoses(
    const sensor_msgs::Image &depth_image,
    const sensor_msgs::CameraInfo &camera_info) {
  // clean up the buffer
  std::for_each(buffer_.begin(), buffer_.end(),
                [](CellContent &value) { value = CellContent::Unknown; });

  // add depth information with no offset
  addDepthInformation(depth_image, camera_info, buffer_, 0.0);

  // Determine which cells can potentially be tomato cans standing on
  // a tray.
  for (int32_t linear_index = static_cast<int32_t>(buffer_.size()) - 1;
       linear_index >= 0; --linear_index) {
    const auto uvw = linearIndexToBufferIndex(linear_index);

    if ((uvw.v < cans_height_ - 1) && (uvw.w > 0)) {
      const auto index_of_cell_below = linearIndexOfElementBelow(linear_index);
      const auto index_of_cell_below_in_front =
          linearIndexOfElementInFront(index_of_cell_below);

      auto is_an_empty_cell = (buffer_[linear_index] == CellContent::Empty);

      auto below_and_below_in_front_are_support =
          (buffer_[index_of_cell_below] == CellContent::Obstacle) &&
          (buffer_[index_of_cell_below_in_front] == CellContent::Obstacle);

      auto the_one_below_is_a_tomato_can =
          (buffer_[index_of_cell_below] == CellContent::TomatoCan);

      if (is_an_empty_cell && (below_and_below_in_front_are_support ||
                               the_one_below_is_a_tomato_can)) {
        buffer_[linear_index] = CellContent::TomatoCan;
      }
    }
  }

  // crappy filter to filter out some undesirable solutions
  for (int32_t linear_index = 0;
       linear_index < static_cast<int32_t>(buffer_.size()); ++linear_index) {
    if (buffer_[linear_index] != CellContent::Obstacle) {
      continue;
    }

    auto slices = 0;
    {
      auto index_of_cell_above = linearIndexOfElementAbove(linear_index);
      while ((index_of_cell_above >= 0) &&
             (buffer_[index_of_cell_above] == CellContent::TomatoCan)) {
        ++slices;
        index_of_cell_above = linearIndexOfElementAbove(index_of_cell_above);
      }
    }

    if (slices < minimum_slices_for_usable_space) {
      auto index_of_cell_above = linearIndexOfElementAbove(linear_index);
      while (slices > 0) {
        buffer_[index_of_cell_above] = CellContent::DoNotUse;
        index_of_cell_above = linearIndexOfElementAbove(index_of_cell_above);
        --slices;
      }
    }
  }

   auto count_tomato_can_stocking_target_poses =
      [](const std::vector<CellContent> &buffer) {
        int32_t tomato_can_stocking_target_poses = 0;
        auto counter_lambda =
            [&tomato_can_stocking_target_poses](const CellContent &cell_value) {
              tomato_can_stocking_target_poses +=
                  (cell_value == CellContent::TomatoCan) ? 1 : 0;
            };
        std::for_each(buffer.begin(), buffer.end(), counter_lambda);
        return tomato_can_stocking_target_poses;
      };

  // refine vertical position, to minimize height to tray
  double y_offset = 0.0;
  double delta{1.1 * cell_height};

 // backup the current buffer
  auto buffer_backup = buffer_;

  const auto tomato_can_stocking_target_poses_count =
      count_tomato_can_stocking_target_poses(buffer_);
  ROS_ERROR_STREAM(
      "Baseline tomato can count: " << tomato_can_stocking_target_poses_count);

  while ((tomato_can_stocking_target_poses_count > 0) && (delta > 0.001)) {
    // apply the depth image, with offset, and count the tomato cans again
    addDepthInformation(depth_image, camera_info, buffer_, y_offset + delta);
    const auto new_tomato_can_stocking_target_poses_count =
        count_tomato_can_stocking_target_poses(buffer_);

    ROS_ERROR_STREAM("Updated tomato can count: "
                     << new_tomato_can_stocking_target_poses_count);
    // if the number of tomato cans has decreased, then we hit the tray
    if (tomato_can_stocking_target_poses_count !=
        new_tomato_can_stocking_target_poses_count) {
      // reduce the delta value
      delta = delta / 2.0;
      ROS_ERROR_STREAM("Delta value too large, reducing to: " << delta);
    } else {
      y_offset += delta;
      ROS_ERROR_STREAM("Updated y_offset value to: " << y_offset);
    }
  }
  ROS_ERROR_STREAM(
      "End of offset optimization, final y_offset value: " << y_offset);

  // Add obstacles
  buffer_ = std::move(buffer_backup);

  auto is_a_valid_tomato_can_pose = [&, this](const int32_t linear_index) {
    bool valid_pose{true};
    const auto uvw = linearIndexToBufferIndex(linear_index);

    // Check the limits
    if ((uvw.v < cans_height_ - can_slices) && (uvw.v > 0)) {
      auto linear_index_slice = linear_index;
      for (int32_t i = 0; i < can_slices; ++i) {
        if (buffer_[linear_index_slice] != CellContent::TomatoCan) {
          valid_pose = false;
          break;
        }
        linear_index_slice = linearIndexOfElementAbove(linear_index_slice);
      }
    } else {
      valid_pose = false;
    }

    // if the pose is valid, mark it as used
    if (valid_pose) {
      auto linear_index_slice = linear_index;
      for (int32_t i = 0; i < can_slices; ++i) {
        buffer_[linear_index_slice] = CellContent::Obstacle;
        linear_index_slice = linearIndexOfElementAbove(linear_index_slice);
      }
    }

    return valid_pose;
  };

  std::vector<geometry_msgs::PoseStamped> tomato_can_stocking_target_poses;
  for (int32_t linear_index = buffer_.size() - 1; linear_index >= 0;
       --linear_index) {
    if (is_a_valid_tomato_can_pose(linear_index)) {
      auto tomato_can_pose =
          bufferIndexToCenterPose(linearIndexToBufferIndex(linear_index));
      tomato_can_pose.position.y += y_offset;

      geometry_msgs::PoseStamped tomato_can_stamped_pose;
      tomato_can_stamped_pose.header.frame_id = depth_image.header.frame_id;
      tomato_can_stamped_pose.pose = tomato_can_pose;

      tomato_can_stocking_target_poses.push_back(tomato_can_stamped_pose);
    }
  }

  return tomato_can_stocking_target_poses;
}

void ShelfScanner::addDepthInformation(
    const sensor_msgs::Image &depth_image,
    const sensor_msgs::CameraInfo &camera_info,
    std::vector<CellContent> &buffer, const double y_offset) const {
  std::this_thread::sleep_for(std::chrono::seconds{1});

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);

  auto cv_image = cv_bridge::toCvCopy(depth_image);

  // backproject the uv coordinates to xyz relative to the camera
  // and mark the cells as an obstacle
  for (int32_t u = 0; u < cv_image->image.cols; ++u) {
    for (int32_t v = 0; v < cv_image->image.rows; ++v) {
      const auto depth = cv_image->image.at<float>(v, u);

      if (std::isnan(depth)) {
        continue;
      }

      cv::Point2d uv_coords;
      uv_coords.x = u;
      uv_coords.y = v;

      auto vec = model.projectPixelTo3dRay(uv_coords);

      // mark as empty all the cells in the ray path to the hit
      for (double raycast_depth = 0.0; raycast_depth < depth;
           raycast_depth += 0.05) {
        // TODO At least in simulation, dont do " / cv::norm(vec)".
        // Depth is the z value, not the distance along the ray
        auto xyz_coords = raycast_depth * vec;
        xyz_coords.y -= y_offset;

        if (isWithinBuffer(xyz_coords.x, xyz_coords.y, xyz_coords.z)) {
          // fill with obstacles all the way back for each (u,v) pair
          auto buffer_index =
              pointToBufferIndex(xyz_coords.x, xyz_coords.y, xyz_coords.z);
          const auto linear_index = bufferIndexToLinearIndex(buffer_index);

          if (buffer[linear_index] == CellContent::Unknown) {
            buffer[linear_index] = CellContent::Empty;
          }
        }
      }

      // mark the occupied cells
      auto xyz_coords = depth * vec;
      xyz_coords.y -= y_offset;

      if (isWithinBuffer(xyz_coords.x, xyz_coords.y, xyz_coords.z)) {
        // fill with obstacles all the way back for each (u,v) pair
        auto buffer_index =
            pointToBufferIndex(xyz_coords.x, xyz_coords.y, xyz_coords.z);

        // mark the one we hit, and the ones behind
        for (; buffer_index.w < cans_depth_; ++buffer_index.w) {
          const auto linear_index = bufferIndexToLinearIndex(buffer_index);
          buffer[linear_index] = CellContent::Obstacle;
        }
      }
    }
  }
}

}  // namespace ek_challenger
