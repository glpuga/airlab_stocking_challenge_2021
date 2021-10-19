/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/console.h>

// project
#include <ek_challenger/arm_joints_pose.hpp>
#include <ek_challenger/nodes/get_arm_joints_for_pose_action_node.hpp>
#include <ek_challenger/data_conversions.hpp>

namespace ek_challenger
{

    BT::NodeStatus GetArmJointsForPoseActionNode::tick()
    {
        std::string pose;
        if (!getInput<std::string>("pose", pose))
        {
            throw BT::RuntimeError("missing required input [pose]");
        }

        if (stored_poses_.find(pose) == stored_poses_.end())
        {
            throw BT::RuntimeError("Unknown pose name: " + pose);
        }

        setOutput<ArmJointsPose>("joints_pose", stored_poses_[pose]);
        return BT::NodeStatus::SUCCESS;
    }

    void GetArmJointsForPoseActionNode::halt()
    {
        _halt_requested = true;
    }
}