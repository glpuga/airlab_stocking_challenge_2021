/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// third party
#include <geometry_msgs/Pose2D.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

// project
#include <ek_challenger/arm_joints_pose.hpp>

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

    template <>
    inline ek_challenger::ArmJointsPose convertFromString(StringView key)
    {
        auto parts = BT::splitString(key, ';');
        if (parts.size() != 7)
        {
            throw BT::RuntimeError("invalid input)");
        }
        else
        {
            ek_challenger::ArmJointsPose output;
            output.arm_joint_1_deg = convertFromString<double>(parts[0]);
            output.arm_joint_2_deg = convertFromString<double>(parts[1]);
            output.arm_joint_3_deg = convertFromString<double>(parts[2]);
            output.arm_joint_4_deg = convertFromString<double>(parts[3]);
            output.arm_joint_5_deg = convertFromString<double>(parts[4]);
            output.arm_joint_6_deg = convertFromString<double>(parts[5]);
            output.arm_joint_7_deg = convertFromString<double>(parts[6]);
            return output;
        }
    }

} // end namespace BT
