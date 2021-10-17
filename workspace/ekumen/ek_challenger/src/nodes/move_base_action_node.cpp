/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// project
#include <ek_challenger/nodes/move_base_action_node.hpp>
#include <ek_challenger/data_conversions.hpp>

namespace ek_challenger
{

    BT::NodeStatus MoveBaseActionNode::tick()
    {
        geometry_msgs::Pose2D goal;
        if (!getInput<geometry_msgs::Pose2D>("goal", goal))
        {
            throw BT::RuntimeError("missing required input [goal]");
        }

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

        ROS_INFO("Waiting for action server to start.");

        ac.waitForServer();

        move_base_msgs::MoveBaseGoal action_goal;

        tf2::Quaternion q;
        q.setRPY(0, 0, goal.theta);

        action_goal.target_pose.header.frame_id = "map";
        action_goal.target_pose.pose.position.x = goal.x;
        action_goal.target_pose.pose.position.y = goal.y;
        action_goal.target_pose.pose.position.z = 0;
        action_goal.target_pose.pose.orientation.x = q.x();
        action_goal.target_pose.pose.orientation.y = q.y();
        action_goal.target_pose.pose.orientation.z = q.z();
        action_goal.target_pose.pose.orientation.w = q.w();

        ROS_INFO_STREAM("Action server started, sending goal: " << action_goal);

        ac.sendGoal(action_goal);

        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (!finished_before_timeout)
        {
            ROS_INFO("Action did not finish before the time out.");
            ac.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        return BT::NodeStatus::SUCCESS;
    }

    void MoveBaseActionNode::halt()
    {
        _halt_requested = true;
    }

}