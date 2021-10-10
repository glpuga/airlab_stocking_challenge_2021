# Changelog

### Oct 10 - Added BehaviorTree.CPP to third-party

- Added the BT library to third-party. The same "Wno-error=shadow" fix was necessary here as with the MTC.
- The version added matches main in the BT repo, plus a number of unpublished patches to add mocking to subtrees for ease of testing.
- Added Groot as well, current main in repo.

To test both running launch Groot in a terminal, selecting "monitor"
```
roslaunch groot Groot
```
then in a second terminal launch the only test that publishes over zmq to Groot.
```
rosrun behaviortree_cpp_v3 t05_cross_door 
```
As soon as the test starts, click "Connect" in the Groot window.

### Oct 10 - Add MoveIt Task Constructor to third-party

Added moveit task constructor to the third_party folder.
To get it to build in melodic and with the gcc version in the container, it required adding "Wno-error=shadow" to all the cmakelists.
After that it builds fine and can run the [moveit demo here](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html).

In a terminal:
```
roslaunch moveit_task_constructor_demo demo.launch
```
then in a second terminal,
```
roslaunch moveit_task_constructor_demo pickplace.launch
```