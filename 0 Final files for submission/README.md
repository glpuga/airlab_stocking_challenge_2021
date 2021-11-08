# AIRLab Delft Stocking Challenge - Submission information

# Intro

This is the submission information for Team Ekumen, on the day of the final deadline for the challenge.

This is the second submission we make. We made an incomplete submission for the deadline of November 1st. For that deadline unfortunately we had not completed an end-to-end solution to the challenge in time, but we had worked hard on the challenge and we wanted to share how far we had made it into the solution by submitting our code and video.

However, after the organization invited the participants to keep submitting solutions until a new deadline set on November 8th, we made the effort to complete the missing functionality before this new deadline. The submission files and media (code + youtube video + this file) have been updated to reflect that.

While we had not managed to complete functionality for the deadline of Nov 1st, we have made good progress since then and we can now submit a version that can perform product stocking.

## Video of the submission at work

Link to the video on youtube: https://youtu.be/N2B1DwXFJD4

The video is also present in a file within `Final files for submission` in this repository.

## General approach

We chose the following strategy:

1. The robot goes to the table and performs segmentation on the depth data to calculate the pose of the tomato cans.
3. The robot then loads a few tomato cans on the tray on its back, for transport to the shelves.
4. The robot identifies the location of the shelf where the cans should be stocked.
5. The robot then transfers the cans to the shelf.
6. The process gets repeated as many times as necessary.

## Contents of the submission

Our code, on the date of Nov 8th, can perform all 6 items in the list above. Our approach still needs tunning, and the behavior is not totally repeatable from run to run.

The location of the table and the general location of the shelves are hardcoded in the robot behavior file loaded with BehaviorTree.CPP. The products on the table and the location of the particular shelf to stock, on the other hand, are found by the robot using sensor data.

Initially we would have wanted to detect the table and the shelf to stock using image recognition, but unfortunately this feature has not been completed in time for this submission. Currently the location of the table is hardcoded on the behavior file, which is an xml file that encodes the behavior of the robot that solves this challenge. This file is a text file, however, and can therefore be easily updated.

The location of the shelf to stock is detected using sensors, but lacking image recognition to detect the tomato cans on the shelf, we implemented the detection of the shelf using a heuristic algorithm that will try to stock any shelf where two levels of tomato cans can be stacked one top of another.

## Tools used

To implement our solution to the challenge we used the following tools:

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP). Awesome tool. It allowed to easily update the competition behavior of the robot, and also experiment different approaches and and run quick tests by composing complex behaviors from simple actions.
- [MoveIt Task Constructor](https://github.com/ros-planning/moveit_task_constructor). A tool with great potential that allowed us to code complex arm movements with smooth transitions between them and fast planning time.
- While we developed pointcloud segmentation using PCL to detect the tomato cans on the table, the code version that is currently integrated in this submission uses OpenCV to perform this action.
- Generally ROS packages (MoveIt, MoveBase) and some additional libraries (OpenCV, fmt).

At some point we trained a YOLO neural network to detect tomato cans on images using [Darknet](https://github.com/AlexeyAB/darknet), and we created a ROS package to help us create a dataset for this by taking multitude of pictures of the tomato can Gazebo model from different angles. This effort, however, got deprioritized in favor of robot behavior and arm planning, so none of this is currently used in the code we submitted but you'll find folders with darknet configuration files in them.

## Usage of the submitted software

To build the development docker, within this repo do:

1. Go into the `docker/` folder.
2. Run `./build.bash` to create the container (this only needs to be done the first time)
3. Run `./run.bash` to start the container and open a terminal to it.
4. Once within, run `catkin_make`, then `source devel/setup.bash`.
5. Then start the simulation with `roslaunch retail_store_simulation tiago_simulation.launch`

That will launch only the competition simulation. To run our code, in a second terminal do:

1. Go into the `docker/` folder.
2. Run `./join.bash` to open a second terminal within the same container.
3. Do `source devel/setup.bash`
4. Finally  `roslaunch ek_challenger ek_challenger.launch`

Our solution to the challenge will begin running, hopefully similarly to what can be seen in the video.

## Where's the code?

The code can be found within the `workspace/ekumen/` folder in this repository. except for a few early tools and scripts, all of it is within the `ek_challenger` package in that folder.

## Will it run in the actual robot?

It's unlikely. The location of the table and the general location of the shelf are hardcoded in the behavior tree, so it's not really adaptable to a different scenario from the one in Gazebo. This is trivial to change, though





