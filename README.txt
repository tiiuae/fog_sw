
ROS2 install:
=============

## Install ROS2 following instructions here:
## https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Binary/

sudo apt install openjdk-11-jdk libasio-dev python3-pip
pip3 install --user pyros-genmsg

# Add ros2 script start into startup script (e.g. ~/.bashrc)
source /opt/ros/foxy/setup.bash


PX4 & gazebo install:
=====================
sudo apt install gazebo9 libgazebo9-dev libgstreamer-plugins-base1.0-dev python3-toml python3-jinja2


Install & Build DroneSW:
========================

# Clone repositories:
pushd .
git clone git@github.com:ssrc-tii/fog_sw.git
cd fog_sw
git submodule update --init --recursive
cd ..
git clone git@github.com:ssrc-tii/px4-firmware.git
cd px4-firmware
git submodule update --init --recursive
popd


# Install tools:
pushd .
cd fog_sw/tools
sudo dpkg -i fastrtps*.deb
popd


# build ros2 modules:
pushd.
cd fog_sw/ros2_ws
colcon build
popd


Start drone simulation:
=======================

# start ros2 mavlink_ctrl
# (Start new terminal window)
cd fog_sw/ros2_ws
source install/setup.bash
ros2 launch px4_mavlink_ctrl mavlink_ctrl.launch

# build & start PX4
# (Start new terminal window)
cd px4-firmware
make px4_sitl_rtps gazebo_solo
# (wait until PX4 starts up..)
micrortps_client start -t UDP


Enable FastRTPS bridge:
=======================

# start fastRTPS agent in ROS2
# (Start new terminal window)
cd fog_sw/ros2_ws/install/px4_ros_cmd/bin
./micrortps_agent -t UDP

# Visualize combined sensor messages transferred from PX4 to ROS2 via RTPS
# (Start new terminal window)
source fog_sw/ros2_ws/install/setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py


