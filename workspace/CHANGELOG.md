# Changelog

### Oct 24 - Multiple new features

New code adds:

- Task constructor planning!
- concurrent execution/planning with for arm movements! (gripper, joint pose planning, task constructor planning).
- Clean up of node names and BTs.

Also added the `fmt` library to use in the task Constructor code. Because of the current issues with the apt package sources from PAL, the library was added as a cmake dependency project.

### Oct 17 - Added a simple autonomy module based on BehaviorTree.CPP

Added a simple BT to use as base for more complex behaviors. For the time being it only has a node to sleep for a number of seconds, and another to move the robot to a target pose using move base.

To launch on a demo, launch the simulation (both gazebo and ek_challengers.launch extra nodes) and then execute:

```
roslaunch ek_challenger autonomy.launch
```

The BT can be monitored using BT. For that open a new terminal, run:
```
roslaunch groot Groot
```
and then click "Connect".

### Oct 17 - Added darknet and darknet ros

Added three modules

- A synthetic dataset generator for darknet.
- A new docker container to train a network on that data.
- darknet_ros in the devel container to detect object on images based on that network.

Unfortunately, darknet_ros does not seem to play ball with the PAL base container, rendering it almost unusable.

Also while investigating that, it was found that dataset generated with the dataset generator led to overfitting. This is properly discussed in an issue in the repository with some ideas to improve this if the problem with darknet_ros is solved.

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