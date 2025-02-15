#!/usr/bin/env bash

# only set environment variables if the Jetson and router IPs are reachable
if ping -c 1 192.168.0.105 &> /dev/null
then
  echo "Jetson IP reachable."
  # set ROS_IP to the IP adress of this host machine
  IP=$(ip addr show | grep wlp | grep inet | awk '{print $2}' | awk 'BEGIN{OFS=FS="/"};NF--');
  if [ -n "$IP" ]; then
      export ROS_IP=$IP;
  else
      export ROS_IP=192.168.0.105;
  fi

  # master node is run on Jetson
  export ROS_MASTER_URI=http://192.168.0.105:11311
else
    echo "Jetson IP not reachable - not setting environment variables."
fi

