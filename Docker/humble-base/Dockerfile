# Building Humble Container
FROM osrf/ros:humble-desktop

# Setup Base Configurations 
RUN apt-get update && apt-get install -y locales\
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en     

# Due to ROS 2 being ran on multiple ws, might have to config in future
WORKDIR /auv
COPY /colcon_ws ./colwon_ws

# Performs Check on Environment Variables for Humble (UNIX-systems)
RUN printenv | grep -i ROS

# Sources config files on terminal bootup by adding to .bashrc
RUN echo "source ../opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ../ros_entrypoint.sh" >> ~/.bashrc