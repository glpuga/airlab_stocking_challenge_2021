cmake_minimum_required(VERSION 3.0.2)
project(ek_challenger)

add_compile_options(-std=c++14 -Wall -Wextra -Werror -Wno-error=shadow)

find_package(catkin REQUIRED COMPONENTS
  octomap_ros
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  roscpp
  rospy
  tf2
)
catkin_package(
  INCLUDE_DIRS
    include
# LIBRARIES ek_challenger
  CATKIN_DEPENDS
    behaviortree_cpp_v3
    moveit_ros_planning_interface
    octomap_ros
    roscpp
    rospy
    tf2
# DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(bt_challenger
  src/bt_challenger.cpp
  src/nodes/get_arm_joints_for_pose_action_node.cpp
  src/nodes/gripper_control_action_node.cpp
  src/nodes/head_control_action_node.cpp
  src/nodes/move_base_action_node.cpp
  src/nodes/set_arm_joints_pose_action_node.cpp
  src/nodes/torso_control_action_node.cpp
  src/nodes/pause_action_node.cpp
)
target_link_libraries(bt_challenger
  ${catkin_LIBRARIES}
)
add_dependencies(bt_challenger
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ek_challenger_node
  src/ek_challenger_node_runner.cpp
)
target_link_libraries(ek_challenger_node
  bt_challenger  
  ${catkin_LIBRARIES}
)
add_dependencies(ek_challenger_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ek_challenger.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
