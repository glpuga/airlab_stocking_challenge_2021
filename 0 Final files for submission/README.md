# AIRLab Delft Stocking Challenge - Submission information

# Intro

This is the submission information for Team Ekumen, on the day of the final deadline for the challenge.

Unfortunately we did not manage to have an end-to-end solution ready to compete in time, so we are aware that our submission is null.

However, we worked hard on it, and we wanted to share what little we managed to get working with you as sign of gratitude for having accepted our participation.

## Video of the submission at work

Link to the video: https://youtu.be/axz91ZYtzio

## General approach

We chose the following strategy:

1. The robot would go to the table
2. It would then identify the location of the cans.
3. It would load as many tomato cans on its back as we could make fit.
4. It would then navigate to the shelf, and identify the locations to stock the cans.
5. It would then transfer the cans from its back to the shelf.
6. Then it would repeat as many times as necessary.

## Contents of the submission

Our current state, on the moment of the deadline, is that we can pick-up the cans from the table and we can place them in the tray on the back of the robot. For that we:

- Navigate to the table using the map.
- Identify the location of the cans from the depth image
- Pick and place the cans using a backported version of the [MoveIt Task Constructor](https://github.com/ros-planning/moveit_task_constructor) (originally it only supports ROS Noetic and onwards), giving us a very fluid secuence of movements for the pick-and-place task.
- We can navigate to the shelves.
- We can scan the shelves, locating potential spots to stock the cans in.
- We sequence all of this using [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)

We barely managed to start integration of all the software pieces the day before the deadline, so all of this kind-of works... most of the time. Well, maybe not most, but works sometimes.

Which gets us to the reason we were unable to complete the challenge: **Unfortunately we could not figure why moveit refuses to pick the cans from the tray of the robot and place them within the shelf at the designated spots, and this prevents us from completing the challenge sucessfully.** Most likely we need some more tuning of MoveIt, but we could not solve it in time for the deadline.

## Tools used

As said before, we use:

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP). Awesome tool
- [MoveIt Task Constructor](https://github.com/ros-planning/moveit_task_constructor). Great tool.
- While we developed pointcloud segmentation using PCL to detect the tomato cans, the version that is currently integrated in the submission uses OpenCV.

At some point we trained a YOLO neural network to detect tomato cans, and created a ROS package to help us create a dataset for this by taking multitude of pictures of the tomato can model from different angles. This effort, however, got deprioritized in favor of behavior and picking, so none of this is currently used in the code we submitted but you'll find folders with darknet configuration files in them.

## Usage of the submitted software

To build the development docker, within this repo do:

1. Go into the `docker/` folder.
2. Run `./build.bash` to create the container (this only needs to be done the first time)
3. Run `./run.bash` to start the container and open a terminal to it.
4. Once within, run `catkin_make`, then `source devel/setup.bash`.
5. Then start the simulation with `roslaunch retail_store_simulation tiago_simulation.launch`

That will launch the competition simulation. Then open a second terminal and do:

1. Go into the `docker/` folder.
2. Run `./join.bash` to open a second terminal within the same container.
3. Do `source devel/setup.bash`
4. Finally  `roslaunch ek_challenger ek_challenger.launch`

Our solution to the challenge will begin running, hopefully similarly to what can be seen in the video.

## Where's the code?

The code can be found within the `workspace/ekumen/` folder in this repository. except for a few early tools and scripts, all of it is within the `ek_challenger` package in that folder.

## Will it run in the actual robot?

Most definitely not. The location of the table and the shelves is hardcoded in the behavior tree. This is trivial to change, but given that the stacking step of the algorithm does not work it's unnecessary to try. Also, we did not get to finely tune obstacle detection using moveit/octomap.

Busy as we were getting to the deadline, I'm afraid we did not prepare a ready-to-run docker with the software already built to run on the robot.




