FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.3

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p ws/src

# #####
#
# Team dependencies start here
#

ARG DIST=melodic

# Base system tools
RUN apt update \
 && apt install -y \
    build-essential \
    clang-format \
    curl \
    cmake \
    lsb-release \
    gdb \
    git \
    locate \
    mc \
    python3-pip \
    sudo \
    tmux \
    wget \
    xterm \
 && apt clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Base ROS melodic system
COPY keys/ros.key /tmp/ros.key
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key add /tmp/ros.key \
 && apt update \
 && apt install -y \
    # ros-${DIST}-velodyne-simulator \
 && rosdep init \
 && apt clean

# [[[ Team dependencies end here (move quick additions above before submission) ]]]

# #####
#
# Fixuid configuration
#

# Set USER and GROUP
ARG USER=user
ARG GROUP=host_group

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

# RUN addgroup --gid 1000 $USER && \
#     adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

RUN mkdir -p /home/$USER/ws/src \
 && chown -R $USER: /home/$USER/ws

# #####
#
# Quick package additions go here
#

# Add extra ROS packages here
RUN apt update \
 && apt install -y \
    ros-${DIST}-moveit-resources-panda-moveit-config \
    ros-${DIST}-rosparam-shortcuts \
 && apt clean

# Install extra tools tools
RUN apt update \
 && apt install -y \
    # tmux \
 && apt clean 

# #####
#
# Final image start setup
#

USER $USER:$GROUP
RUN rosdep update

# Build and source your ros packages 
WORKDIR /home/user/sim_ws
RUN rm -rf /home/user/sim_ws/devel \
 && rm -rf /home/user/sim_ws/build \
 && rm -rf /home/user/sim_ws/logs \
 && bash -c "source /opt/pal/ferrum/setup.bash && catkin_make" \
 && echo 'source /home/user/sim_ws/devel/setup.bash' >> ~/.bashrc 

# Source all the needed environment files.
RUN /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'

# We can't use WORKDIR for this final adjustment because the directory
# is set in .bashrc in the base image
RUN echo 'cd /home/user/ws' >> ~/.bashrc 

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
