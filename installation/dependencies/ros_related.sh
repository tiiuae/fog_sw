#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: installing ROS2-related dependencies"

sudo apt-get -y install \
  ros-foxy-octomap \
  ros-foxy-octomap-msgs \
  ros-foxy-dynamic-edt-3d \

