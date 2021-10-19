/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <cmath>

// third-party
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>

// project
#include <ek_challenger/nodes/head_control_action_node.hpp>

namespace ek_challenger
{

    BT::NodeStatus HeadControlActionNode::tick()
    {
        double pitch;
        double yaw;
        if (!getInput<double>("pitch", pitch))
        {
            pitch = 0.0;
        }
        if (!getInput<double>("yaw", yaw))
        {
            yaw = 0.0;
        }

        actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("/head_controller/point_head_action", true);

        ROS_INFO("Waiting for action server to start.");

        ac.waitForServer();

        control_msgs::PointHeadGoal action_goal;

        constexpr auto to_rad = [](const double deg)
        {
            return 3.14159 / 180.0 * deg;
        };

        // to approximate a neck orientation using a vector from baselink, we use a really long baseline
        constexpr double r = 100;
        const auto distance = r * std::cos(to_rad(pitch));
        const auto elevation = r * std::sin(to_rad(pitch));

        action_goal.target.point.x = distance * std::sin(to_rad(yaw));
        action_goal.target.point.y = elevation;
        action_goal.target.point.z = distance * std::cos(to_rad(yaw));
        action_goal.target.header.frame_id = "base_link";

        action_goal.pointing_axis.x = 0;
        action_goal.pointing_axis.y = 0;
        action_goal.pointing_axis.z = 1;

        action_goal.pointing_frame = "xtion_rgb_optical_frame";
        action_goal.min_duration = ros::Duration(1);
        action_goal.max_velocity = 1.0;

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

    void HeadControlActionNode::halt()
    {
        _halt_requested = true;
    }

}