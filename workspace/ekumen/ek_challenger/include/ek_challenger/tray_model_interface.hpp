/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// ros
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

namespace ek_challenger {

class TrayModelInterface {
 public:
  using Ptr = std::unique_ptr<TrayModelInterface>;

  enum class Side { LEFT, RIGHT };

  virtual ~TrayModelInterface() = default;

  virtual geometry_msgs::PoseStamped trayPose() const = 0;

  virtual void clear() = 0;

  virtual std::string addLocus(const geometry_msgs::PoseStamped &absolute_pose,
                               const bool occupied, const int32_t) = 0;

  virtual std::string addLocus(const geometry_msgs::Pose2D &relative_pose,
                               const bool occupied,
                               const int32_t fill_priority) = 0;

  virtual bool pickEmptyLocus(const Side &from_side, std::string &locus_id) = 0;

  virtual bool pickOccupiedLocus(const Side &from_side,
                                 std::string &locus_id) = 0;

  virtual void setLocusState(const std::string &locus_id,
                             const bool occupied) = 0;

  virtual void releaseLocus(const std::string &locus_id) = 0;

  virtual const geometry_msgs::PoseStamped getLocusPose(
      const std::string &locus_id) const = 0;

  virtual void updatePlanningScene() = 0;
};

}  // namespace ek_challenger
