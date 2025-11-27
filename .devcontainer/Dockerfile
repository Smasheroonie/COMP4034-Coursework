# FROM ros:humble
FROM my-devcontainer-image:latest

ARG USERNAME=team-beta
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user without sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Set environment for ROS 2
ENV SHELL=/bin/bash

# Pre-install system dependencies and ROS packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv\
    libopencv-dev\
    net-tools \
    nano \
    iputils-ping \
    curl \
    fswebcam \
    nano \
    gnupg2 \
    lsb-release \
    sudo \
    build-essential \
    mesa-utils\
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-gazebo-ros-pkgs\
    ros-humble-gazebo-plugins \
    ros-humble-cv-bridge \
    ros-humble-rqt* \
    # ros-humble-gazebo-*\
    # ros-humble-ros-gz\
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install pyrealsense2
# RUN mkdir -p /home/ros2_ws/src && cd /home/ros2_ws/src && \
#     [ -d turtlebot3 ] || git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
#     [ -d turtlebot3_msgs ] || git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
#     [ -d DynamixelSDK ] || git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
#     [ -d turtlebot3_simulations ] || git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    
# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN usermod -aG video team-beta
# Set default user
USER $USERNAME

CMD ["/bin/bash"]
