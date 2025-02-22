# Base ROS 2 Humble Container
FROM osrf/ros:humble-desktop

# Setup UTF-8 Locale
RUN apt-get update && apt-get install -y locales \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# Install Required Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    build-essential

# Create the colcon workspace and clone micro-ROS repositories
WORKDIR /colcon_ws/src
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git

# Install dependencies
WORKDIR /colcon_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y"

# Ensure the install directory exists
RUN mkdir -p /colcon_ws/install

# Build micro-ROS packages
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Ensure the micro-ROS setup script is sourced in new shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /colcon_ws

# Run a bash shell by default
CMD ["bash"]
