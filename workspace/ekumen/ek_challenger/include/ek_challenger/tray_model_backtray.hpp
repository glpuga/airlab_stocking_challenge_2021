/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

#include <string>

// project
#include <ek_challenger/tray_model_impl.hpp>

namespace ek_challenger {

class TrayModelBacktray : public TrayModelImpl {
 public:
  TrayModelBacktray(const std::string &name,
                    const geometry_msgs::PoseStamped &pose,
                    const std::vector<std::string> &moveit_namespaces)
      : TrayModelImpl(name, pose, moveit_namespaces) {}

 private:
  void updateSceneAddingFrame(CollisionObjectManager &om) override {
    auto pose1 = trayPose();
    auto pose2 = trayPose();
    const auto distance = 0.70;
    pose1.pose.position.y -= distance;
    pose2.pose.position.y += distance;

    om.addBox("guard1", pose1, 1.6, 0.02, 1);
    om.addBox("guard2", pose2, 1.6, 0.02, 1);
  }
};

}  // namespace ek_challenger
