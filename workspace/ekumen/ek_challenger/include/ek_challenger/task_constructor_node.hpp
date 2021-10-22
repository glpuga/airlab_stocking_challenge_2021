/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// third party
#include <ros/ros.h>

// project
#include <ek_challenger/TaskConstructorExec.h>
#include <ek_challenger/TaskConstructorPlan.h>

#include <ek_challenger/tasks/pick_place_task.hpp>

namespace ek_challenger {

class TaskConstructorNode {
 public:
  TaskConstructorNode();

  bool run();

 private:
  ros::NodeHandle nh_;

  ros::ServiceServer plan_srv_;
  ros::ServiceServer exec_srv_;

  std::unique_ptr<PickPlaceTask> task_ptr_;

  bool planCallback(ek_challenger::TaskConstructorPlan::Request &req,
                    ek_challenger::TaskConstructorPlan::Response &res);

  bool execCallback(ek_challenger::TaskConstructorExec::Request &req,
                    ek_challenger::TaskConstructorExec::Response &res);
};

}  // namespace ek_challenger
