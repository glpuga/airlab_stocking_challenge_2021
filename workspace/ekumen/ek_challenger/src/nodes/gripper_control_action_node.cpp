/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <moveit/move_group_interface/move_group_interface.h>

// project
#include <ek_challenger/nodes/gripper_control_action_node.hpp>

namespace ek_challenger
{

    BT::NodeStatus GripperControlActionNode::tick()
    {
        std::string move_group;
        std::string apperture;
        if (!getInput<std::string>("move_group", move_group))
        {
            throw BT::RuntimeError("missing required input [move_group]");
        }
        if (!getInput<double>("apperture", apperture))
        {
            throw BT::RuntimeError("missing required input [apperture]");
        }

        ROS_INFO("Creating move group");

        auto move_group_ptr_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                move_group);
        move_group_ptr_->setMaxAccelerationScalingFactor(1.0);
        move_group_ptr_->setMaxVelocityScalingFactor(1.0);

        ROS_INFO_STREAM("Action server started, sending goal: " << action_goal);

        move_group_ptr->setJointValueTarget(joint_group_positions);

        move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
    }

    INFO("Closest-safe-spot movement: {} is planning", name());
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
        auto success = (move_group_ptr->plan(movement_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ERROR("{} failed to generate a plan", name());
            return false;
        }
    }

    INFO("Closest-safe-spot movement: {} is executing", name());
    {
        auto success = (move_group_ptr->execute(movement_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ERROR("{} failed to move execute", name());
            return false;
        }
    }

    return BT::NodeStatus::SUCCESS;
}

void GripperControlActionNode::halt()
{
    _halt_requested = true;
}
}