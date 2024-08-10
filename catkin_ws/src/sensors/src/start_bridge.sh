#!/bin/bash


CMD="source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && source /set_env.bash && ros2 run ros1_bridge dynamic_bridge"
echo $CMD

# --bridge-all-2to1-topics

docker exec -it jetson-bridge-1 bash -c "$CMD"