# Changelog

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