/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <map>
#include <string>

// third-party
#include <ek_challenger/tray_model_interface.hpp>

namespace ek_challenger {

using TrayData = std::map<std::string, std::shared_ptr<TrayModelInterface>>;

}