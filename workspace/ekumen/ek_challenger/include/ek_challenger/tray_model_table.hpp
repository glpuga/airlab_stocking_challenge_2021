/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

#include <string>

// project
#include <ek_challenger/tray_model_impl.hpp>

namespace ek_challenger {

class TrayModelTable : public TrayModelImpl {
 public:
  TrayModelTable(const std::string &name,
                 const geometry_msgs::PoseStamped &pose,
                 const std::vector<std::string> &moveit_namespaces)
      : TrayModelImpl(name, pose, moveit_namespaces) {}
};

}  // namespace ek_challenger
