FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Base dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    python3-pip \
    git \
    libdw-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install "numpy<2.0" mujoco

# backward-cpp for segfault debugging
RUN wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp && \
    mv backward.hpp /usr/include

# Clone sdk_deploy repo
WORKDIR /workspace
RUN git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

# Fix eigen submodule (replace GitLab with GitHub mirror to avoid SSL timeout)
RUN cd sdk_deploy && \
    find . -name ".gitmodules" | xargs grep -l "eigen" 2>/dev/null | \
    xargs sed -i 's|https://gitlab.com/libeigen/eigen.git|https://github.com/live-clones/eigen.git|g' 2>/dev/null || true

# Initialize submodules
RUN cd sdk_deploy && \
    git submodule sync && \
    git submodule update --init --recursive

# Install libevdev-dev (needed by lite3_sdk_deploy for -levdev linker flag)
# Separate layer — keeps everything above cached on rebuild
RUN apt-get update && apt-get install -y libevdev-dev && rm -rf /var/lib/apt/lists/*

# Build with ROS2 colcon
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd sdk_deploy && \
    colcon build \
    --packages-up-to lite3_sdk_deploy \
    --cmake-args -DBUILD_PLATFORM=x86"

# Source ROS2 automatically in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/sdk_deploy/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /root/.bashrc

WORKDIR /workspace/sdk_deploy
