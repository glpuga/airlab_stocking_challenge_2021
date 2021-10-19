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
#include <ek_challenger/nodes/torso_control_action_node.hpp>

namespace ek_challenger
{

    BT::NodeStatus TorsoControlActionNode::tick()
    {
        double relative_height;
        if (!getInput<double>("relative_height", relative_height))
        {
            throw BT::RuntimeError("missing required input [relative_height]");
        }

        ROS_INFO("Creating move group");

        auto move_group_ptr =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(torso_move_group_name_);
        move_group_ptr->setMaxAccelerationScalingFactor(1.0);
        move_group_ptr->setMaxVelocityScalingFactor(1.0);

        std::vector<double> joint_group_positions;

        const robot_state::JointModelGroup *joint_model_group =
            move_group_ptr->getCurrentState()->getJointModelGroup(torso_move_group_name_);

        auto current_state = move_group_ptr->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group,
                                               joint_group_positions);

        ROS_INFO_STREAM("Current state of the torso joint is " << joint_group_positions[0]);

        joint_group_positions[0] = (torso_max_range_ - torso_min_range_) * relative_height + torso_min_range_;

        move_group_ptr->setJointValueTarget(joint_group_positions);
        move_group_ptr->setStartState(*current_state);

        ROS_INFO_STREAM("Target state of the torso joint is " << joint_group_positions[0]);

        ROS_INFO_STREAM("Planning gripper motion for group " << name());

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

        ROS_INFO_STREAM("Executing gripper motion for group " << name());
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

    void TorsoControlActionNode::halt()
    {
        _halt_requested = true;
    }
}