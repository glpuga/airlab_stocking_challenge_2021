/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// third-party
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>

// project
#include <ek_challenger/data_conversions.hpp>
#include <ek_challenger/nodes/create_obstacle_action_node.hpp>

namespace ek_challenger {

BT::NodeStatus CreateObstacleActionNode::tick() {
  std::string object_id;
  if (!getInput<std::string>("object_id", object_id)) {
    throw BT::RuntimeError("missing required input [object_id]");
  }

  std::string ns;
  if (!getInput<std::string>("ns", ns)) {
    // default to root namespace
    ns = "";
        throw BT::RuntimeError("missing required input [ns]");
  }

  geometry_msgs::PoseStamped pose;
  if (!getInput<geometry_msgs::PoseStamped>("pose", pose)) {
    throw BT::RuntimeError("missing required input [pose]");
  }

  ros::NodeHandle nh{ns};
  ros::ServiceClient planning_scene_diff_client =
      nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = object_id;

  {
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.14;  // height
    primitive.dimensions[1] = 0.03;  // width
    collision_object.primitives.push_back(primitive);
  }

  {
    geometry_msgs::Pose primitive_pose;
    primitive_pose.orientation.w = 1.0;
    primitive_pose.position.x = pose.pose.position.x;
    primitive_pose.position.y = pose.pose.position.y;
    primitive_pose.position.z = pose.pose.position.z;
    collision_object.primitive_poses.push_back(primitive_pose);
  }

  collision_object.operation = collision_object.ADD;

  {
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;

    planning_scene_diff_client.waitForExistence();
    planning_scene_diff_client.call(srv);
  }

  return BT::NodeStatus::SUCCESS;
}

void CreateObstacleActionNode::halt() { _halt_requested = true; }

}  // namespace ek_challenger