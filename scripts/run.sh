#!/bin/bash

PACKAGE=$1
LAUNCH_FILE=$2

if [ "$#" -ne 2 ] || [ -z "$PACKAGE" ] || [ -z "$LAUNCH_FILE" ]; then
    echo "Usage: $0 <PACKAGE> <LAUNCH_FILE>"
    exit 1
fi

cd ../Docker/jetson/
echo "Starting docker container..."
OVERRIDE_COMMAND="source /opt/ros/noetic/setup.bash && source /AUV-2025/catkin_ws/devel/setup.bash && roslaunch $PACKAGE $LAUNCH_FILE" docker compose up --d