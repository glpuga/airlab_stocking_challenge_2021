/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>
#include <vector>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/console.h>

// project
#include <ek_challenger/arm_joints_pose.hpp>
#include <ek_challenger/nodes/set_arm_joints_pose_action_node.hpp>
#include <ek_challenger/data_conversions.hpp>

namespace ek_challenger
{

    BT::NodeStatus SetArmsJointPoseActionNode::tick()
    {
        ArmJointsPose joints;
        if (!getInput<ArmJointsPose>("joints_pose", joints))
        {
            throw BT::RuntimeError("missing required input [joints_pose]");
        }

        ROS_INFO("Creating move group");

        auto move_group_ptr =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                move_group_);
        move_group_ptr->setMaxAccelerationScalingFactor(1.0);
        move_group_ptr->setMaxVelocityScalingFactor(1.0);

        std::vector<double> joint_group_positions;

        const robot_state::JointModelGroup *joint_model_group =
            move_group_ptr->getCurrentState()->getJointModelGroup(move_group_);

        auto current_state = move_group_ptr->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group,
                                               joint_group_positions);

        ROS_DEBUG_STREAM("Current state of the joints is " << joint_group_positions[0] << " " << joint_group_positions[1]);

        auto to_rad = [](const double deg)
        { return 3.14159 / 180.0 * deg; };

        joint_group_positions[0] = to_rad(joints.arm_joint_1_deg);
        joint_group_positions[1] = to_rad(joints.arm_joint_2_deg);
        joint_group_positions[2] = to_rad(joints.arm_joint_3_deg);
        joint_group_positions[3] = to_rad(joints.arm_joint_4_deg);
        joint_group_positions[4] = to_rad(joints.arm_joint_5_deg);
        joint_group_positions[5] = to_rad(joints.arm_joint_6_deg);
        joint_group_positions[6] = to_rad(joints.arm_joint_7_deg);

        move_group_ptr->setJointValueTarget(joint_group_positions);
        move_group_ptr->setStartState(*current_state);

        ROS_DEBUG_STREAM("Target state of the joints is " << joint_group_positions[0] << " " << joint_group_positions[1]);

        ROS_INFO_STREAM("Planning motion for group " << name());

        moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
        {
            auto success = (move_group_ptr->plan(movement_plan) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_ERROR_STREAM(name() << " failed to generate a plan");
                return BT::NodeStatus::FAILURE;
            }
        }

        ROS_INFO_STREAM("Executing motion for group " << name());
        {
            auto success = (move_group_ptr->execute(movement_plan) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_ERROR_STREAM(name() << " failed to execute plan");
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

    void SetArmsJointPoseActionNode::halt()
    {
        _halt_requested = true;
    }
}