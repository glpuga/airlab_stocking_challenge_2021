/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <thread>

// ros
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

// project
#include <logger/logger.hpp>
#include <tijcore/utils/angles.hpp>
#include <tijros/PickAndPlaceRobotCommonImpl.hpp>
#include <tijros/utils/utils.hpp>

namespace tijros {

namespace {

// TODO(glpuga) this frame id is hardcoded in multiple places. Use the one that
// can be get through the scene configuration in  the toolbox
static char world_frame[] = "world";

static const double landing_pose_height = 0.30;
static const double pick_search_length = 0.06;
static const double part_drop_height = 0.04;
static const double twist_height_correction = 0.17;

static const double pickup_displacement_jump_threshold = 10.0;
static const double pickup_displacement_step = 0.0025;
static const double pickup_displacement_step_div = 15;
static const double max_planning_time = 20.0;
static const int max_planning_attempts = 5;

static const double tight_goal_position_tolerance = 0.001;
static const double tight_goal_orientation_tolerance =
    tijcore::utils::angles::degreesToRadians(0.2);

static const double coarse_goal_position_tolerance = 0.015;
static const double coarse_goal_orientation_tolerance =
    tijcore::utils::angles::degreesToRadians(2.5);

moveit_msgs::CollisionObject
createCollisionBox(const std::string &id, const std::string &sub_id,
                   const std::string &frame_id, const double width,
                   const double height, const double depth,
                   const double xoffset, const double yoffset,
                   const double zoffset,
                   const int operation = moveit_msgs::CollisionObject::ADD) {
  // do the hard part
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id + "_" + sub_id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = width;
  primitive.dimensions[1] = height;
  primitive.dimensions[2] = depth;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = xoffset;
  box_pose.position.y = yoffset;
  box_pose.position.z = zoffset;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = operation;

  return collision_object;
}

double estimatePartHeight(const tijcore::Matrix3 &orientation_in_world,
                          const tijcore::PartTypeId &id) {
  const auto part_dimensions = tijcore::part_type::dimensions(id);

  const double estimated_height = std::abs(tijcore::Vector3{0, 0, 1}.dot(
      part_dimensions[0] * orientation_in_world.col(0) +
      part_dimensions[1] * orientation_in_world.col(1) +
      part_dimensions[2] * orientation_in_world.col(2)));
  return estimated_height;
}

} // namespace

PickAndPlaceRobotCommonImpl::PickAndPlaceRobotCommonImpl(
    const tijcore::Toolbox::SharedPtr &toolbox)
    : toolbox_{toolbox} {}

void PickAndPlaceRobotCommonImpl::configureGoalTolerances(
    const bool tight_mode) const {
  if (tight_mode) {
    move_group_ptr_->setGoalPositionTolerance(tight_goal_position_tolerance);
    move_group_ptr_->setGoalOrientationTolerance(
        tight_goal_orientation_tolerance);
  } else {
    move_group_ptr_->setGoalPositionTolerance(coarse_goal_position_tolerance);
    move_group_ptr_->setGoalOrientationTolerance(
        coarse_goal_orientation_tolerance);
  }
}

moveit::planning_interface::MoveGroupInterface *
PickAndPlaceRobotCommonImpl::getMoveItGroupHandlePtr() const {
  // if the setup had not been performed before, do it now
  // Ugly AF, need to find a better solution.

  // TODO(glpuga) for some reason, keeping state in this object causes
  // errors in one of the joints, in both robots(!)
  const auto custom_moveit_namespace = "/ariac/custom/" + name();

  move_group_ptr_.reset();
  if (!move_group_ptr_) {
    moveit::planning_interface::MoveGroupInterface::Options options{
        getRobotPlanningGroup(), custom_moveit_namespace + "/robot_description",
        ros::NodeHandle(custom_moveit_namespace)};
    move_group_ptr_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            options);
    move_group_ptr_->setPlanningTime(max_planning_time);
    move_group_ptr_->setNumPlanningAttempts(max_planning_attempts);

    // default to coarse tolerances
    configureGoalTolerances(false);

    // moveit seems to be defaulting to a value less than one now...
    move_group_ptr_->setMaxAccelerationScalingFactor(1.0);
    move_group_ptr_->setMaxVelocityScalingFactor(1.0);
  }

  if (!planning_scene_ptr_) {
    planning_scene_ptr_ =
        std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
            custom_moveit_namespace);
    setupObjectConstraints();
  }

  return move_group_ptr_.get();
}

bool PickAndPlaceRobotCommonImpl::getInSafePose() const {
  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Get-in-rest-pose movement: {} is calcularting the target", name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    patchJointStateValuesForRestingPose(joint_group_positions);
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Get-in-rest-pose movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to execute", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} finished execution", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getInSafePoseNearTarget(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Get-in-rest-pose movement: {} is calcularting the target", name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,

                                           joint_group_positions);
    patchJointStateValuesForRestingPose(joint_group_positions);
    patchJointStateValuesToGetCloseToTarget(joint_group_positions, target);

    move_group_ptr->setJointValueTarget(joint_group_positions);

    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Get-in-rest-pose movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to execute", name());
      return false;
    }
  }

  INFO("Get-in-rest-pose movement: {} finished execution", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getToGraspingPoseHint(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }

  return true;

  auto move_group_ptr = getMoveItGroupHandlePtr();

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  INFO("Closest-safe-spot movement: {} is calcularting the target joint for "
       "resting pose",
       name());
  {
    moveit::core::RobotStatePtr current_state =
        move_group_ptr->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,
                                           joint_group_positions);

    patchJointStateValuesForRestingPose(joint_group_positions);
    patchJointStateValuesGraspingHingPoseNearTarget(joint_group_positions,
                                                    target);

    move_group_ptr->setJointValueTarget(joint_group_positions);

    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
  }

  INFO("Closest-safe-spot movement: {} is planning", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan", name());
      return false;
    }
  }

  INFO("Closest-safe-spot movement: {} is executing", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to move execute", name());
      return false;
    }
  }

  INFO("Closest-safe-spot movement: {} plan complete", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::getInLandingSpot(
    const tijcore::RelativePose3 &target) const {
  if (!enabled()) {
    return false;
  }

  auto move_group_ptr = getMoveItGroupHandlePtr();
  move_group_ptr->setPoseReferenceFrame(world_frame);

  // Convert the target pose to the world reference frame, and set the
  // orientation pointing to the part
  auto frame_transformer = toolbox_->getFrameTransformer();
  auto end_effector_target_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto approximation_pose_in_world = end_effector_target_pose;
  approximation_pose_in_world.position().vector().z() += landing_pose_height;

  INFO("Approximation pose at {}: {} ", world_frame,
       approximation_pose_in_world);

  INFO("Approximation movement: {} is generating the moveit plan", name());
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
  {
    auto target_geo_pose =
        utils::convertCorePoseToGeoPose(approximation_pose_in_world.pose());
    move_group_ptr->setPoseTarget(target_geo_pose);
    move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

    bool success = (move_group_ptr->plan(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to generate a plan to arrive at the approximation pose",
            name());
      return false;
    }
  }

  INFO("Approximation movement: {} is executing the movement", name());
  {
    auto success = (move_group_ptr->execute(movement_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ERROR("{} failed to move arm into the approximation pose", name());
      return false;
    }
  }

  INFO("{} arrived at the approximation pose for ", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::graspPartFromAbove(
    const tijcore::RelativePose3 &target,
    const tijcore::PartTypeId &part_type_id) const {
  if (!enabled()) {
    return false;
  }

  auto move_group_ptr = getMoveItGroupHandlePtr();

  INFO("Pick-up movement: {} is calculating the pickup trajectory", name());

  // configure tighter tolerances for this
  configureGoalTolerances(true);

  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);
  setSuctionGripper(true);

  auto frame_transformer = toolbox_->getFrameTransformer();

  const auto target_in_world_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);

  // TODO(glpuga) this should better be an function that given the part pose,
  // returns the estimated gripper pose
  auto end_effector_target_pose = target_in_world_pose;
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  // notice that part poses get detected with a height equal to about half the
  // height of the piece
  const auto run_top =
      end_effector_target_pose_in_world.position().vector().z() +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(),
                         part_type_id) *
          0.5 +
      pick_search_length * 0.5;
  const auto run_bottom =
      end_effector_target_pose_in_world.position().vector().z() +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(),
                         part_type_id) *
          0.5 -
      pick_search_length * 0.5;
  end_effector_target_pose_in_world.position().vector().z() = run_top;

  double part_displacement = 0.0;

  while (!gripperHasPartAttached() &&
         (end_effector_target_pose_in_world.position().vector().z() >
          run_bottom)) {
    std::vector<geometry_msgs::Pose> waypoints;

    {
      // TODO(glpuga) conveyor belt hack
      // update the y coordinate because the target may be moving on a
      // conveyor belt
      auto current_target_pose =
          frame_transformer->transformPoseToFrame(target, world_frame);
      end_effector_target_pose_in_world.position().vector().y() =
          current_target_pose.position().vector().y();
    }

    // decrease the gripper one step
    end_effector_target_pose_in_world.position().vector().z() -=
        pickup_displacement_step;

    INFO("Pick-up movement: {} is generating the moveit plan", name());
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
      auto target_geo_pose_in_world =
          utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world);
      move_group_ptr->setPoseTarget(target_geo_pose_in_world);
      move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

      bool success = (move_group_ptr->plan(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success) {
        ERROR("{} failed to generate a plan", name());
        return false;
      }
    }

    INFO("Pick-up movement: {} is executing the moveit plan", name());

    // TODO(glpuga) conveyor belt hack
    // adjust trajectory to account for planning time and part speed

    auto current_target_pose =
        frame_transformer->transformPoseToFrame(target, world_frame);

    part_displacement +=
        end_effector_target_pose_in_world.position().vector().y() -
        current_target_pose.position().vector().y();

    auto &trajectory = movement_plan.trajectory_;
    auto t0 = trajectory.joint_trajectory.points[0].time_from_start.toSec();

    const auto &joint_names = trajectory.joint_trajectory.joint_names;
    for (uint32_t j = 0; j < joint_names.size(); ++j) {
      if (joint_names[j] != "linear_arm_actuator_joint") {
        continue;
      }
      auto &points = trajectory.joint_trajectory.points;
      for (uint32_t i = 1; i < points.size(); ++i) {
        points[i].positions[j] -= part_displacement;

        if (end_effector_target_pose_in_world.position().vector().x() > -1) {
          auto t1 =
              trajectory.joint_trajectory.points[i].time_from_start.toSec();
          points[i].positions[j] -= 0.2 * (t1 - t0);
        }
      }
    }

    INFO("Pick-up movement: {} is executing the movement", name());
    move_group_ptr->execute(trajectory);
  }

  if (gripperHasPartAttached()) {
    INFO("{} successfully grasped a part!", name());
    return true;
  }

  ERROR("{} failed to pick a part", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::placePartFromAbove(
    const tijcore::RelativePose3 &target,
    const tijcore::PartTypeId &part_type_id) const {
  if (!enabled()) {
    return false;
  }
  auto move_group_ptr = getMoveItGroupHandlePtr();

  INFO("Place movement: {} is calculating the pickup trajectory", name());

  // make sure we trade in world poses only
  move_group_ptr->setPoseReferenceFrame(world_frame);

  auto frame_transformer = toolbox_->getFrameTransformer();

  const auto target_in_world_pose =
      frame_transformer->transformPoseToFrame(target, world_frame);

  // TODO(glpuga) this should better be a function that given the part pose,
  // retursns the estimated grasping pose
  auto end_effector_target_pose = target_in_world_pose;
  alignEndEffectorWithTarget(end_effector_target_pose);

  auto end_effector_target_pose_in_world = end_effector_target_pose.pose();

  end_effector_target_pose_in_world.position().vector().z() +=
      part_drop_height +
      estimatePartHeight(target_in_world_pose.rotation().rotationMatrix(),
                         part_type_id);

  {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(
        utils::convertCorePoseToGeoPose(end_effector_target_pose_in_world));

    INFO("Place will drop the part once we are at {} in {} frame",
         utils::convertGeoPoseToCorePose(waypoints.at(0)), world_frame);

    INFO("Place movement: {} is generating the moveit plan", name());
    moveit_msgs::RobotTrajectory trajectory;
    {
      const double fraction = move_group_ptr->computeCartesianPath(
          waypoints, pickup_displacement_step,
          pickup_displacement_jump_threshold, trajectory);
      DEBUG("Place movement: cartesian planner plan fraction for {}: {}",
            name(), fraction);
      bool success = (fraction > 0.95);
      if (!success) {
        ERROR("{} failed to generate a plan to the drop point "
              "(fraction: {})",
              name(), fraction);
        WARNING("{} will release the part here");
        setSuctionGripper(false);
        return false;
      }
    }

    // we don't check the return value because it's going to return failure once
    // we hit the table
    INFO("Place movement: {} is executing the movement", name());
    move_group_ptr->execute(trajectory);
  }

  setSuctionGripper(false);
  INFO("{} placed the part that the destination", name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::twistPartInPlace(
    tijcore::RelativePose3 &target,
    const tijcore::PartTypeId &part_type_id) const {
  using tijcore::utils::angles::degreesToRadians;

  if (!enabled()) {
    return false;
  }

  auto move_group_ptr = getMoveItGroupHandlePtr();

  move_group_ptr->setPoseReferenceFrame(world_frame);

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(
          getRobotPlanningGroup());

  auto frame_transformer = toolbox_->getFrameTransformer();

  double estimated_part_height;
  {
    // determine the part height
    const auto target_in_world_frame =
        frame_transformer->transformPoseToFrame(target, world_frame);
    estimated_part_height = estimatePartHeight(
        target_in_world_frame.rotation().rotationMatrix(), part_type_id);
  }

  // we determine the rotation that goes from the end effector frame rotation to
  // the target rotation, to update the rotation of the part once we have
  // changed the orientation of the gripper
  tijcore::Matrix3 target_in_end_effector_rotation;
  {
    const auto rotated_target_rotation_matrix =
        target.rotation().rotationMatrix();

    const auto end_effector_pose_in_world =
        utils::convertGeoPoseToCorePose(move_group_ptr->getCurrentPose().pose);
    const auto end_effector_pose_in_target_frame =
        frame_transformer->transformPoseToFrame(
            tijcore::RelativePose3{world_frame, end_effector_pose_in_world},
            target.frameId());
    const auto end_effector_rotation_matrix_in_target_frame =
        end_effector_pose_in_target_frame.rotation().rotationMatrix();

    target_in_end_effector_rotation =
        end_effector_rotation_matrix_in_target_frame.inv() *
        rotated_target_rotation_matrix;
  }

  INFO("Twist-part-in-place movement: {} is planning to align the end efector "
       "with the arm articulations",
       name());
  {
    {
      moveit::core::RobotStatePtr current_state =
          move_group_ptr->getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group,
                                             joint_group_positions);
      patchJointStateValuesForAlignedZeroWrist(joint_group_positions);
      move_group_ptr->setJointValueTarget(joint_group_positions);
      move_group_ptr->setStartState(*move_group_ptr->getCurrentState());
    }

    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
      auto success = (move_group_ptr->plan(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success) {
        ERROR("{} failed to execute the plan", name());
        return false;
      }
    }

    INFO("Twist-part-in-place movement: {} is executing the movement to align "
         "the end efector with the arm articulations",
         name());
    {
      auto success = (move_group_ptr->execute(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success) {
        ERROR("{} failed to execute end effector twist", name());
        return false;
      }
    }
  }

  {
    INFO("Twist-part-in-place movement:: {} is planning to twist the end "
         "effector",
         name());

    auto rotated_end_effector_in_world =
        utils::convertGeoPoseToCorePose(move_group_ptr->getCurrentPose().pose);

    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    {
      {
        auto rotated_target_rotation_matrix =
            rotated_end_effector_in_world.rotation().rotationMatrix();

        rotated_target_rotation_matrix *=
            tijcore::Rotation::fromRollPitchYaw(0, 0, degreesToRadians(-90))
                .rotationMatrix();

        rotated_end_effector_in_world.rotation() =
            tijcore::Rotation(rotated_target_rotation_matrix);
        rotated_end_effector_in_world.position().vector() +=
            rotated_target_rotation_matrix.col(1) * twist_height_correction +
            rotated_target_rotation_matrix.col(0) *
                (-estimated_part_height / 2.0);
      }

      move_group_ptr->setPoseTarget(
          utils::convertCorePoseToGeoPose(rotated_end_effector_in_world));
      move_group_ptr->setStartState(*move_group_ptr->getCurrentState());

      bool success = (move_group_ptr->plan(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!success) {
        ERROR("{} failed to generate a plan to twist the end effector", name());
        return false;
      }
    }

    INFO("Twist-part-in-place movement: {} is executing the twist movement",
         name());
    {
      auto success = (move_group_ptr->execute(movement_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // TODO(glpuga) with the competition controller parameters, loading the
      // gripper from the side causes the goal tolerances to be exceeded.
      success = true;

      if (!success) {
        ERROR("{} failed to execute end effecto twist", name());
        return false;
      }
    }

    // update the target locus orientation based on the orientation of the
    // gripper
    {
      INFO("Twist-part-in-place movement: {} is updating the locus orientation",
           name());

      // get the current pose of the end effector in the frame of the target
      // pose
      auto end_effector_pose_in_target_frame =
          frame_transformer->transformPoseToFrame(
              tijcore::RelativePose3{world_frame,
                                     rotated_end_effector_in_world},
              target.frameId());

      // get the rotation matrix
      auto end_effector_rotation_matrix_in_target_frame =
          end_effector_pose_in_target_frame.rotation().rotationMatrix();

      // given that we know the rotation between the end effector and the part,
      // use that to infer the rotation part now.
      auto new_target_orientation =
          end_effector_rotation_matrix_in_target_frame *
          target_in_end_effector_rotation;

      target.rotation() = tijcore::Rotation{new_target_orientation};
    }
  }

  INFO("{} completed the twist movement at the approximation pose for ",
       name());
  return true;
}

bool PickAndPlaceRobotCommonImpl::dropPartWhereYouStand() const {
  setSuctionGripper(false);
  INFO("{} turned the suction gripper off", name());
  return true;
}

void PickAndPlaceRobotCommonImpl::cancelAction() {
  // TODO(glpuga) This will is a half-assed solution to this,
  // and will probably only work "most" of the time.
  // move_group_ptr->stop();
}

void PickAndPlaceRobotCommonImpl::setupObjectConstraints() const {
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  const int operation = moveit_msgs::CollisionObject::ADD;

  auto scene_configuration = toolbox_->getSceneConfigReader();
  const double z_offset{0.025};

  DEBUG(" Generating collision scene");
  DEBUG(" - adding AGV tray representatives");
  for (const auto &item : scene_configuration->getListOfAgvs()) {
    collision_objects.push_back(
        createCollisionBox(item.name, "surface", item.frame_id, 0.5, 0.7,
                           z_offset, 0, 0, 0, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "tower_foot", item.frame_id, 0.23, 0.23,
                           0.22, 0.0, -0.45, 0.11, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "tower_head", item.frame_id, 0.15, 0.15,
                           0.7, 0.0, -0.45, 0.35, operation));
  }

  DEBUG(" - adding assembly station representatives");
  for (const auto &item : scene_configuration->getListOfAssemblyStations()) {
    collision_objects.push_back(
        createCollisionBox(item.name, "briefcase_table", item.frame_id, 1.6,
                           1.2, z_offset, -0.3, -0.1, 0.0, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "briefcase_front", item.frame_id, 0.05,
                           0.6, 0.16, 0.3, 0.0, 0.08, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "briefcase_side", item.frame_id, 0.6,
                           0.05, 0.16, 0.0, -0.3, 0.08, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "table_right", item.frame_id, 1.0, 0.10,
                           1.0, -0.70, -0.75, 0.5, operation));
    collision_objects.push_back(createCollisionBox(item.name, "table_back",
                                                   item.frame_id, 0.8, 1.2, 1.0,
                                                   -0.7, -0.1, 0.5, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "table_left", item.frame_id, 1.6, 0.1,
                           1.0, -0.45, 0.54, 0.5, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "briefcase_top_1", item.frame_id, 0.04,
                           0.1, 0.80, 0.33, 0.35, 0.40, operation));
    collision_objects.push_back(createCollisionBox(item.name, "briefcase_top_2",
                                                   item.frame_id, 0.7, 0.1, 0.1,
                                                   0.0, 0.35, 0.75, operation));
  }

  DEBUG(" - adding bin representatives");
  for (const auto &item : scene_configuration->getListOfBins()) {
    collision_objects.push_back(
        createCollisionBox(item.name, "surface", item.frame_id, 0.6, 0.6,
                           z_offset, 0, 0, 0, operation));

    collision_objects.push_back(
        createCollisionBox(item.name, "wall_1", item.frame_id, 0.64, 0.06, 0.08,
                           0.0, 0.31, 0.04, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "wall_2", item.frame_id, 0.64, 0.06, 0.08,
                           0.0, -0.31, 0.04, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "wall_3", item.frame_id, 0.06, 0.64, 0.08,
                           0.31, 0.0, 0.04, operation));
    collision_objects.push_back(
        createCollisionBox(item.name, "wall_4", item.frame_id, 0.06, 0.64, 0.08,
                           -0.31, 0.0, 0.04, operation));
  }

  DEBUG(" - adding conveyor belt representatives");
  for (const auto &item : scene_configuration->getListOfConveyorBelts()) {
    collision_objects.push_back(
        createCollisionBox(item.name, "surface", item.container_frame_id, 0.63,
                           9, z_offset, 0, 0, 0, operation));
  }

  // kitting rail
  collision_objects.push_back(createCollisionBox(
      "kitting", "rail", "world", 0.4, 10.0, 0.10, -1.3, 0.0, 0.93, operation));

  // Imaginary divider
  collision_objects.push_back(createCollisionBox(
      "divider", "rail", "world", 0.05, 10.0, 0.1, -1.4, 0.0, 2.7, operation));

  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotCommonImpl::markAsCommanded(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors,
    const int command) {

  // TODO(glpuga) improve this
  // this method call is needed to generate the pointers if planning_scene_ptr_
  // has not been populated yet.
  getMoveItGroupHandlePtr();

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (const auto &item : descriptors) {
    const auto &frame_id = item.center.frameId();
    const auto &offsets = item.center.position().vector();
    collision_objects.push_back(createCollisionBox(
        "access", item.id, frame_id, item.x_size, item.y_size, item.z_size,
        offsets.x(), offsets.y(), offsets.z(), command));
  }
  planning_scene_ptr_->applyCollisionObjects(collision_objects);
}

void PickAndPlaceRobotCommonImpl::markAsInaccessible(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors) {
  markAsCommanded(descriptors, moveit_msgs::CollisionObject::ADD);
}

void PickAndPlaceRobotCommonImpl::markAsAccessible(
    const std::vector<tijcore::ModelTraySharedAccessSpaceDescription>
        &descriptors) {
  markAsCommanded(descriptors, moveit_msgs::CollisionObject::REMOVE);
}

void PickAndPlaceRobotCommonImpl::alignEndEffectorWithTarget(
    tijcore::RelativePose3 &end_effector_target_pose) const {
  const auto orientation = end_effector_target_pose.rotation().rotationMatrix();
  const auto x_director = tijcore::Vector3{0, 0, -1};

  auto original_x_director = orientation.col(0);
  auto original_y_director = orientation.col(1);
  auto original_z_director = orientation.col(2);

  // to try to consistently align with the same axis, which is important for
  // part flipping, try to use always the same vector, unless that's the one
  // that's pointing up.
  tijcore::Vector3 z_director;
  if ((std::abs(x_director.dot(original_x_director)) >
       std::abs(x_director.dot(original_y_director))) &&
      (std::abs(x_director.dot(original_x_director)) >
       std::abs(x_director.dot(original_z_director)))) {
    // x is pointing up
    // TODO(glpuga) the sign inversion is because without this accessing the
    // briefcases to remove a battery fails because the full body of the robot
    // is oriented to try to match the orientation of the piece, resulting in
    // unfeasable plans. It's likely related to the end of range of the wrist
    // articulations.
    z_director = orientation.col(2) * (-1);
  } else {
    // is horizontal, always choose x
    // TODO(glpuga) the sign inversion is because without this accessing the
    // briefcases to remove a battery fails because the full body of the robot
    // is oriented to try to match the orientation of the piece, resulting in
    // unfeasable plans. It's likely related to the end of range of the wrist
    // articulations.
    z_director = orientation.col(0) * (-1);
  }

  // Don't assume the target will be perfectly normal to the world Z axis
  z_director -= x_director * z_director.dot(x_director);
  z_director = z_director / z_director.norm();

  auto y_director = z_director.cross(x_director);
  y_director = y_director / y_director.norm();

  const auto end_effector_orientation =
      tijcore::Matrix3{
          x_director,
          y_director,
          z_director,
      }
          .trans();

  end_effector_target_pose.rotation() =
      tijcore::Rotation{end_effector_orientation};
}

} // namespace tijros
