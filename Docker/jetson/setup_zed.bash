#!/bin/bash
cd /
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git zed
apt-get update
rosdep install --from-paths zed --ignore-src -r -y