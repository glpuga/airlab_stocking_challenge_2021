/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

#include <string>

// project
#include <ek_challenger/tray_model_impl.hpp>

namespace ek_challenger
{

  class TrayModelBacktray : public TrayModelImpl
  {
  public:
    TrayModelBacktray(const std::string &name,
                      const geometry_msgs::PoseStamped &pose,
                      const std::vector<std::string> &moveit_namespaces)
        : TrayModelImpl(name, pose, moveit_namespaces) {}

  private:
    void updateSceneAddingFrame(CollisionObjectManager &) override
    {
    }
  };

} // namespace ek_challenger
