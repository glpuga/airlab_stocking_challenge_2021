FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.3

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<pkg_name> pal-ferrum-<pkg_name> <apt-pkg>

# Build and source your ros packages 
RUN bash -c "source /opt/pal/ferrum/setup.bash \
    && catkin build \
    && echo 'source /opt/pal/ferrum/setup.bash' >> ~/.bashrc"
    # Add below line to automatically source your packages
    # && echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc

ENTRYPOINT ["bash"]

