ARG BASEIMG=ubuntu:hirsute
FROM $BASEIMG

LABEL maintainer="Gerardo Puga <glpuga@gmail.com>"

ARG USER=user
ARG GROUP=user

# #####
#
# Team dependencies start here
#

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Base system tools
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    bash-completion \
    build-essential \
    curl \
    cmake \
    lsb-release \
    git \
    locate \
    mc \
    python3-pip \
    software-properties-common \
    sudo \
    tmux \
    wget \
 && apt clean

 # Install extra tools tools
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    libopencv-dev \
    nvidia-cuda-toolkit \
 && apt clean 

# Install a recent version of cmake, required to be able to build the latest version of Darknet.
# The one already present in the ubuntu repositories is not recent enough.
RUN sudo apt update \
 && sudo apt install -y gpg wget \
 && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
 && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null \
 && sudo apt-get update \
 && sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg \
 && sudo apt-get install -y kitware-archive-keyring \
 && sudo apt-get install -y cmake \
 && apt clean

# Build Darknet from source and install within /opt
WORKDIR /root
RUN git clone https://github.com/AlexeyAB/darknet.git \
 && cd darknet \
 && git checkout 9ff8653d999c8a22bc8f1ff4f4a8a3cc5b63d255 \
 && mkdir -p build_release && cd build_release \
 && cmake .. -DENABLE_CUDNN=OFF \
 && cmake --build . --target install -- -j$(expr `nproc` - 4) \
 && echo "Done!"
RUN mv /root/darknet /opt/darknet

# [[[ Team dependencies end here (move quick additions above before submission) ]]]

# #####
#
# Fixuid configuration
#

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# #####
#
# Quick package additions go here
#

# Install extra tools tools
# RUN export DEBIAN_FRONTEND=noninteractive \
#  && apt update \
#  && apt install -y \
#     libopencv-dev \
#  && apt clean 

# #####
#
# Final image start setup
#

USER $USER:$GROUP

# Download the Yolov4 pretrained network. Needed when training from scratch.
RUN /bin/sh -c 'mkdir -p ~/data/yolo' \
 && cd ~/data/yolo \
 && curl -sL "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.conv.137" -o yolov4.conv.137

# Source all the needed environment files.
RUN /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'

ARG WORKSPACE_PATH=/home/${USER}/workspace

RUN mkdir -p ${WORKSPACE_PATH}

WORKDIR ${WORKSPACE_PATH}

# Add darknet to the path
RUN /bin/sh -c 'echo "export PATH=\"\$PATH\":/opt/darknet/build_release" >> ~/.bashrc'

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
