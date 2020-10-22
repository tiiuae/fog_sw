#!/bin/sh

echo "--- Generating /etc/ros/rosdep/sources.list.d/50-fogsw.list (as su)"
sudo sh -c 'echo "yaml file://${PWD}/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-fogsw.list'

echo "--- Updating rosdep"
rosdep update
