# AIRLab Delft Stocking Challenge - Technical Environment

Welcome to the AIRLab Delft stocking challenge! In this repository you should find all technical details and resources to develop and submit your solution for the competition. It is important that you read this README thoroughly, it will help you getting started as quickly as possible. For more information about rules and regulations of the challenge take a look at the `AIRLab_Delft_Stocking_Challenge_Rules_And_Regulations.pdf`.

## Deliverable guidelines

The deliverable solution will consist of one single file, a docker image.

This docker image will have a catkin workspace inside $HOME/ws/ containing a single launchfile that can be called using `roslaunch stocking_challenge solution.launch`. This launchfile should start the simulation with the solution.

## Getting started

In order to get started working on your solution please follow the following steps:

1. Create a private repository on gitlab.com
2. Add a ws directory in it with a src directory inside. In this src directory you will add your own packages.
3. Add a Dockerfile, use the template in from this repository.
4. Update your docker image following the steps below.
5. Now you can start working in the ws/src folder in your repository.

The directory structure in your repository should looks as follows:

```
.  
├── Dockerfile  
├── ws  
│   └── src  
│       └── <your_ros_pkgs>  
```


## Updating your docker image

There are two main reasons you should update the docker image of the gitlab repository:

1. The competition docker image has changed (you would be notified via email)
2. A team member modified the Dockerfile and it needs to be tested and used by other members

To do so the process should be the following:

``` bash
#Download the latest competition docker image, note you have to substitute <X.X> with the latest version from the container registry.
docker login registry.gitlab.com
docker pull registry.gitlab.com/competitions4/airlab/stocking-challenge:<X.X>
# Update your local private_gitlab containing the latest Dockerfile and the latest version of your code
cd <private_gitlab_path>
git pull --rebase
# Build your image from competition
docker build . --tag registry.gitlab.com/<docker_remote_path_name>
# Upload your new image to private_gitlab
docker push registry.gitlab.com/<docker_remote_path_name>
```

**It is very important that all dependencies are added to the dockerfile!**
Generally, you will need to install ros packages, libraries or programs in your docker to support your own code. If you need a package, you should add the line to install it to the Dockerfile.

Usually, these packages are installed with:

`sudo apt install ros-melodic-<pkg-name>`

To install them in your docker image, you should add the following line in the docker file:

`RUN apt install -y ros-melodic-<pkg_name>`

## Daily usage
On a daily basis, you will work inside of the docker image using a shared directory. Once inside the docker, any file created or any program installed on the docker will be deleted, if the specific file was not saved on a shared directory. If the user needs to install a specific software everytime, it is better to create a new docker following the instructions above, and taking as a base the updated docker image.

In order to start the docker image, you will need to log in to the docker daemon on gitlab.com:   
`$ docker login registry.gitlab.com`    
Then, enter your personal GitLab username and password.

For your convenience, we have published some scripts that simplify the launch of a docker with GPU acceleration. Follow the instructions at [pal_docker_utils](https://github.com/pal-robotics/pal_docker_utils) to properly set up your environment with nvidia-docker. If you do not follow the steps properly, you will not be able to run gazebo, rviz or other graphical applications from within the docker container.

Once logged and after configuring pal_docker_utils, you will need to execute the pal_docker.sh script with the name of the image and the application you want to start.

`$ ~/pal_docker.sh -it -v PATH_TO_YOUR_SHARED_WS PATH_TO_YOUR_DOCKER_IMAGE`

The previous command starts a bash terminal inside a container of the specified image with a shared ws.

Example for this repo (using /$HOME/your_repo_dir/ws/src as shared workspace):

`$ ~/pal_docker.sh -it -v /$HOME/your_repo_dir/ws/src:/home/user/ws registry.gitlab.com/competitions4/airlab/stocking-challenge`

Now, any changes in ws/src will be still be present after you exit the docker image. Note that you still have to commit and push these changes to your repository yourself!

## Launching the simulation
Once you are in the docker image, you can run the simulation using
`$ roslaunch retail_store_simulation tiago_simulation`

You will then see TIAGo spawn in a small gazebo world as described in the challenge document. You can move TIAGo to the table and cabinet using the go_to_poi service, or using the [robot_steering](http://wiki.ros.org/rqt_robot_steering) plugin in rqt.

Launching the simulation will start all necessary topics, services and actions for you to build your solution on top off.
