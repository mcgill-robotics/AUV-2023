FROM dustynv/pytorch:2.1-r35.3.1

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

RUN apt update \
    && apt install -y openssh-server xauth \
    && mkdir /var/run/sshd \
    && mkdir /root/.ssh \
    && chmod 700 /root/.ssh \
    && echo "Port 222" >> /etc/ssh/sshd_config \
    && echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
    && echo 'root:jetson' | chpasswd
    
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV IGNITION_VERSION=fortress

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN apt-get update && apt-get install -y wget \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip tmux vim

RUN apt-get install -y ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-smach-ros \
    ros-noetic-usb-cam \
    ros-noetic-robot-localization

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
            libjpeg-dev \
		  zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt \
    && pip install --no-deps --no-cache-dir ultralytics==8.0.207 \
    && rm -f /usr/local/bin/cmake && rm -rf /root/.cache/pip && rm requirements.txt

RUN git clone --branch release/0.16 --recursive --depth=1 https://github.com/pytorch/vision torchvision && \
    cd torchvision && \
    git checkout release/0.16 && \
    python3 setup.py bdist_wheel && \
    cp dist/torchvision*.whl /opt && \
    pip3 install --no-cache-dir --verbose /opt/torchvision*.whl && \
    cd ../ && \
    rm -rf torchvision \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/scripts/set_env.bash" >> ~/.bashrc \
    && echo "source /AUV-2025/scripts/jetson/docker_aliases.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y tmux vim iputils-ping iproute2 ros-noetic-robot-localization \
    && pip3 install keyboard \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y docker.io && pip install sklearn

RUN echo "tmux" >> ~/.bashrc