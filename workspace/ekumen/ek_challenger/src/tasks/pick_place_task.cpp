/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>
#include <string>
#include <utility>
#include <vector>

// third-party
#include <fmt/format.h>
#include <tf2/LinearMath/Quaternion.h>

// project
#include <ek_challenger/tasks/pick_place_task.hpp>

namespace ek_challenger {

using namespace moveit::task_constructor;

namespace {

moveit_msgs::RobotState buildRobotStateMsg(
    const std::map<std::string, double> &joints) {
  moveit_msgs::RobotState robot_state;
  robot_state.joint_state.name.reserve(joints.size());
  robot_state.joint_state.position.reserve(joints.size());

  for (auto &joint : joints) {
    robot_state.joint_state.name.push_back(joint.first);
    robot_state.joint_state.position.push_back(joint.second);
  }
  robot_state.is_diff = true;
  return robot_state;
}

}  // namespace

PickPlaceTask::PickPlaceTask(const std::string &side) {
  ROS_INFO("PickAndPlace task handler created");

  arm_group_name_ = fmt::format("arm_{}", side);
  hand_group_name_ = fmt::format("gripper_{}", side);
  hand_frame_ = fmt::format("gripper_{}_grasping_frame", side);
  eef_name_ = fmt::format("gripper_{}", side);

  open_hand_joint_values_ = {
      {fmt::format("gripper_{}_left_finger_joint", side), 0.044},
      {fmt::format("gripper_{}_right_finger_joint", side), 0.044}};

  closed_hand_joint_values_ = {
      {fmt::format("gripper_{}_left_finger_joint", side), 0.025},
      {fmt::format("gripper_{}_right_finger_joint", side), 0.025}};

  auto to_rad = [](const double deg) { return 3.14159 * deg / 180.0; };

  // Rest pose:
  //
  resting_arm_joint_poses_ = {
      {fmt::format("arm_{}_1_joint", side), to_rad(-60)},
      {fmt::format("arm_{}_2_joint", side), to_rad(84)},
      {fmt::format("arm_{}_3_joint", side), to_rad(156)},
      {fmt::format("arm_{}_4_joint", side), to_rad(98)},
      {fmt::format("arm_{}_5_joint", side), to_rad(-90)},
      {fmt::format("arm_{}_6_joint", side), to_rad(80)},
      {fmt::format("arm_{}_7_joint", side), to_rad(0)}};

  // Uptray pose :

  // resting_arm_joint_poses_ = {
  //     {fmt::format("arm_{}_1_joint", side), to_rad(60)},
  //     {fmt::format("arm_{}_2_joint", side), to_rad(0)},
  //     {fmt::format("arm_{}_3_joint", side), to_rad(0)},
  //     {fmt::format("arm_{}_4_joint", side), to_rad(90)},
  //     {fmt::format("arm_{}_5_joint", side), to_rad(120)},
  //     {fmt::format("arm_{}_6_joint", side), to_rad(-60)},
  //     {fmt::format("arm_{}_7_joint", side), to_rad(-15)}};

  // Uptray pose:
  //
  // resting_arm_joint_poses_ = {
  //     {fmt::format("arm_{}_1_joint", side), to_rad(60)},
  //     {fmt::format("arm_{}_2_joint", side), to_rad(-30)},
  //     {fmt::format("arm_{}_3_joint", side), to_rad(60)},
  //     {fmt::format("arm_{}_4_joint", side), to_rad(30)},
  //     {fmt::format("arm_{}_5_joint", side), to_rad(-100)},
  //     {fmt::format("arm_{}_6_joint", side), to_rad(0)},
  //     {fmt::format("arm_{}_7_joint", side), to_rad(0)}};

  grasp_frame_transform_.header.frame_id = hand_frame_;
  grasp_frame_transform_.pose.position.x = 0.07;
  grasp_frame_transform_.pose.position.y = 0.0;
  grasp_frame_transform_.pose.position.z = 0.0;
}

bool PickPlaceTask::build(const std::string &object_name,
                          const geometry_msgs::PoseStamped &target_pose,
                          const bool move_to_home, const bool flat_hand_mode) {
  ROS_INFO("Initializing task pipeline");

  auto to_rad = [](const double deg) { return 3.14159 * deg / 180.0; };

  const double grasp_angle = flat_hand_mode ? 0.0 : -5.0;

  const double approach_vector_x_comp = std::cos(to_rad(grasp_angle));
  const double approach_vector_z_comp = -std::sin(to_rad(grasp_angle));

  {
    tf2::Quaternion q;
    q.setRPY(to_rad(0.0), to_rad(grasp_angle), to_rad(0.0));
    grasp_frame_transform_.pose.orientation.x = q.x();
    grasp_frame_transform_.pose.orientation.y = q.y();
    grasp_frame_transform_.pose.orientation.z = q.z();
    grasp_frame_transform_.pose.orientation.w = q.w();
  }

  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  task_->stages()->setName(task_name_);
  task_->loadRobotModel();

  // Sampling planner
  auto sampling_planner =
      std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner =
      std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  task_->setProperty("group", arm_group_name_);
  task_->setProperty("hand", hand_group_name_);
  task_->setProperty("hand_grasping_frame", hand_frame_);
  task_->setProperty("eef", eef_name_);
  task_->setProperty("ik_frame", hand_frame_);

  /* Current State */

  // Forward current_state on to grasp pose generator
  moveit::task_constructor::Stage *current_state_ptr = nullptr;
  {
    auto current_state =
        std::make_unique<stages::CurrentState>("current state");

    // Verify that object is not attached
    auto applicability_filter = std::make_unique<stages::PredicateFilter>(
        "applicability test", std::move(current_state));

    auto predicate_lambda = [object_name](const SolutionBase &s,
                                          std::string &comment) {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(object_name)) {
        comment = "object with id '" + object_name +
                  "' is already attached and cannot be picked";
        return false;
      }
      return true;
    };

    applicability_filter->setPredicate(predicate_lambda);

    current_state_ptr = applicability_filter.get();

    task_->add(std::move(applicability_filter));
  }

  /* Open Hand */

  {
    // Open Hand
    auto stage =
        std::make_unique<stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(hand_group_name_);
    stage->setGoal(open_hand_joint_values_);

    task_->add(std::move(stage));
  }

  /* Move to Pick */

  {
    // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{
                            {arm_group_name_, sampling_planner}});
    stage->setTimeout(connect_planning_timeout_);
    stage->properties().configureInitFrom(Stage::PARENT);

    task_->add(std::move(stage));
  }

  /* Pick Object */

  // Forward attach_object_stage to place pose generator
  moveit::task_constructor::Stage *attach_object_stage = nullptr;
  {
    auto grasp = std::make_unique<SerialContainer>("pick object");

    task_->properties().exposeTo(grasp->properties(),
                                 {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(Stage::PARENT,
                                          {"eef", "hand", "group", "ik_frame"});

    /* Approach Object */

    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(approach_object_min_dist_,
                               approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped direction_vector;
      direction_vector.header.frame_id = hand_frame_;
      direction_vector.vector.x = approach_vector_x_comp;
      direction_vector.vector.z = approach_vector_z_comp;
      stage->setDirection(direction_vector);

      grasp->insert(std::move(stage));
    }

    /* Generate Grasp Pose */

    {
      // Sample grasp pose
      auto stage =
          std::make_unique<stages::GenerateGraspPose>("generate grasp pose");

      stage->properties().configureInitFrom(Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");

      stage->setPreGraspPose(buildRobotStateMsg(open_hand_joint_values_));
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK",
                                                         std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_);

      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    /* Allow Collision (hand/object) */
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collision (hand <-> object)");
      stage->allowCollisions({object_name, "target"},
                             task_->getRobotModel()
                                 ->getJointModelGroup(hand_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    /* Allow collision (object <-> target) */
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collision (object_name,target)");
      stage->allowCollisions({object_name}, {"target"}, true);
      grasp->insert(std::move(stage));
    }

    /* Close Hand */
    {
      auto stage =
          std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(hand_group_name_);
      stage->setGoal(closed_hand_joint_values_);
      grasp->insert(std::move(stage));
    }

    /* Attach Object */
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame_);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /* Allow collision (object/support) */
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collision (object_name,support)");
      stage->allowCollisions({object_name}, support_surfaces_, true);
      grasp->insert(std::move(stage));
    }

    /* Lift object */

    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    /* Forbid collision (object <-> support) */
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid collision (object_name,surface)");
      stage->allowCollisions({object_name}, support_surfaces_, false);
      grasp->insert(std::move(stage));
    }

    // Add grasp container to task
    task_->add(std::move(grasp));
  }

  /* Move to Place */
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to place", stages::Connect::GroupPlannerVector{
                             {arm_group_name_, sampling_planner}});
    stage->setTimeout(connect_planning_timeout_);
    stage->properties().configureInitFrom(Stage::PARENT);
    task_->add(std::move(stage));
  }

  /* Place Object */

  {
    auto place = std::make_unique<SerialContainer>("place object");

    task_->properties().exposeTo(place->properties(), {"eef", "hand", "group"});
    place->properties().configureInitFrom(Stage::PARENT,
                                          {"eef", "hand", "group"});

    /* get in the vertical horizontally */

    {
      auto stage = std::make_unique<stages::MoveRelative>("get in vertical",
                                                          cartesian_planner);

      stage->properties().set("marker_ns", "get_in_the_vertical");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.4);

      // Set motion to place the object
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;

      vec.vector.x = approach_vector_x_comp;
      vec.vector.z = approach_vector_z_comp;

      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /* Lower Object */

    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object",
                                                          cartesian_planner);

      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", world_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.005, 0.15);

      // Set motion to place the object
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1;

      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /* Generate Place Pose  */

    {
      // Generate Place Pose
      auto stage =
          std::make_unique<stages::GeneratePlacePose>("generate place pose");

      stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);

      // TODO adjust place pose based on part height
      // p.pose.position.z += 0.5 * object_dimensions_[0] +
      // place_surface_offset_; TODO TODO TODO TODO TODO TODO TODO TODO TODO

      stage->setPose(target_pose);
      stage->setMonitoredStage(
          attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK",
                                                         std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(grasp_frame_transform_);
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      place->insert(std::move(wrapper));
    }

    /* Open Hand */

    {
      auto stage =
          std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(hand_group_name_);
      stage->setGoal(open_hand_joint_values_);
      place->insert(std::move(stage));
    }

    /* Forbid collision (hand, object) */
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");
      stage->allowCollisions(object_name,
                             task_->getRobotModel()
                                 ->getJointModelGroup(hand_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    /* Detach Object */
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, hand_frame_);
      place->insert(std::move(stage));
    }

    /* lift hand */

    {
      auto stage = std::make_unique<stages::MoveRelative>("up before retreat",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->properties().set("link", world_frame_);
      stage->setMinMaxDistance(0.01, 0.20);
      stage->properties().set("marker_ns", "up_pre_retreat");
      
      // Set motion to place the object
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1;

      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /* Retreat Motion */

    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.10, 0.4);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.x = -approach_vector_x_comp;
      vec.vector.z = -approach_vector_z_comp;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    task_->add(std::move(place));
  }

  /* Move to Home */
  if (move_to_home) {
    auto stage =
        std::make_unique<stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setGoal(resting_arm_joint_poses_);
    stage->restrictDirection(stages::MoveTo::FORWARD);
    task_->add(std::move(stage));
  }

  // prepare Task structure for planning
  try {
    task_->init();
  } catch (InitStageException &e) {
    ROS_ERROR_STREAM("Initialization failed: " << e);
    return false;
  }

  return true;
}

bool PickPlaceTask::plan() {
  ROS_INFO("Start searching for task solutions");
  const int max_solutions = 10;
  return task_->plan(max_solutions);
}

bool PickPlaceTask::execute() {
  ROS_INFO("Executing solution trajectory");
  moveit_msgs::MoveItErrorCodes execute_result;

  execute_result = task_->execute(*task_->solutions().front());

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM(
        "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
}  // namespace ek_challenger
