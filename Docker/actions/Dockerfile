# Use the official ROS Noetic base image
FROM mcgillrobotics/auv_2025:base
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y --no-install-recommends ignition-fortress \
    && apt-get update \
    && apt install -y ros-noetic-ros-ign \
    ros-noetic-ros-ign-bridge \
    ros-noetic-ros-ign-gazebo \
    ros-noetic-gazebo-ros

RUN apt-get -y update \
    && apt-get install -y software-properties-common \
    && apt-get update \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get -y upgrade \
    && apt-get install -y libglapi-mesa xvfb
# && apt-get install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev \
# libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev