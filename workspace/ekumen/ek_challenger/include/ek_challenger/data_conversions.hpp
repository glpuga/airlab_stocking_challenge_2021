/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third party
#include <geometry_msgs/Pose2D.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

namespace BT
{
    template <>
    inline geometry_msgs::Pose2D convertFromString(StringView key)
    {
        auto parts = BT::splitString(key, ';');
        if (parts.size() != 3)
        {
            throw BT::RuntimeError("invalid input)");
        }
        else
        {
            geometry_msgs::Pose2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }
} // end namespace BT
