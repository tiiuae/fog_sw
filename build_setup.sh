#!/bin/sh

echo "--- Generating /etc/ros/rosdep/sources.list.d/50-fogsw.list (as su)"
sudo sh -c 'mkdir -p /etc/ros/rosdep/sources.list.d'
sudo sh -c 'echo "yaml file://${PWD}/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-fogsw.list'

if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
	echo "--- Initialize rosdep"
	rosdep init
fi

echo "--- Updating rosdep"
rosdep update
