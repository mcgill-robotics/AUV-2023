# Building Humble Container
FROM osrf/ros:humble-desktop

# Setup Base Configurations 
RUN apt-get update && apt-get install -y locales\
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# Install Python and pip
RUN apt-get update && \
    apt-get install -y python3 python3-pip python3.10-venv || \
    apt-get install -y python3-venv

# Sources config files on terminal bootup by adding to .bashrc
RUN echo "source ../opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ../ros_entrypoint.sh" >> ~/.bashrc

# Sourcing terminal and copying repos
COPY /colcon_ws ./colcon_ws
WORKDIR /colcon_ws
RUN ../ros_entrypoint.sh colcon build --symlink-install --packages-skip vision

RUN apt-get update && rosdep update && \
    ../ros_entrypoint.sh rosdep install --from-paths src/uros --ignore-src -y 

RUN bash -c "source install/local_setup.bash"

RUN bash -c "../ros_entrypoint.sh colcon build --symlink-install --packages-skip vision"

# Performs Check on Environment Variables for Humble (UNIX-systems)
RUN printenv | grep -i ROS

# RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

ENTRYPOINT ["/ros_entrypoint.sh"]