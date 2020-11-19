# Drone HW setup instructions

Here are step by step instructions to setup ROS2 environment into Drone HW.

## ROS2 install

Install ROS2 following instructions here:<br>
https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

```
$ sudo apt install openjdk-11-jdk libasio-dev python3-pip python3-colcon-common-extensions python3-future
$ pip3 install --user pyros-genmsg
```
Add ROS2 script start into startup script (e.g. ~/.bashrc)<br>
`$ source /opt/ros/foxy/setup.bash`


## PX4 & gazebo install
```
$ sudo apt install gazebo9 libgazebo9-dev libgstreamer-plugins-base1.0-dev python3-toml $ python3-jinja2 libopencv-dev
```
## Install & Build DroneSW

### Clone repositories:
```
$ pushd .
$ git clone git@github.com:ssrc-tii/fog_sw.git
$ cd fog_sw
$ git submodule update --init --recursive
$ cd ..
$ git clone git@github.com:ssrc-tii/px4-firmware.git
$ cd px4-firmware
$ git submodule update --init --recursive
$ popd
```
### Install tools
```
$ pushd .
$ cd fog_sw/tools
$ sudo dpkg -i fastrtps*.deb
$ sudo dpkg -i mavsdk_0.34.0_ubuntu20.04_amd64.deb
$ popd
```
### Update ROS2 dependencies
```
$ pushd .
$ cd fog_sw
$ ./build_setup.sh
$ popd
```
### Build and Generate debian packages
Install tools for debian packaging<br>
`$ sudo apt install python3-bloom dh-make debhelper fakeroot`

Generate deb files:
```
$ source ros2_ws/install/setup.bash
$ cd packaging
$ ./package.sh
```


## Prepare HW
1. Transfer debian packages to the drone mission computer via ssh or USB stick. Following steps expect .deb files being located in **/packages** directory.
2. Install debian packages in drone mission computer:
```
$ cd /packages
$ dpkg -i *.deb
```
3. Recycle drone power or reboot mission computer

______________________________

# Drone Simulation

Easiest way to setup simulation environment is to use container based solution. You can find instructions from another repository: **fog_docker**

In case you need to run some components manually in local PC, you can follow there steps:

### Build ROS2 modules
```
$ pushd .
$ cd fog_sw/ros2_ws
$ colcon build
$ popd
```

### Communication link module
See build & launch instructions from ***ros2_ws/src/communication_link/README.md*** file

### Build & start PX4 flight controller with Gazebo simulation environment
_[ Start new terminal window and run commands: ]_
```
$ cd px4-firmware
$ make px4_sitl_rtps gazebo_iris_rtps
```
_[ Wait until PX4 starts up.. and run command in PX4 shell: ]_
```
pxh> micrortps_client start -t UDP
```
### Launch mavlink_ctrl node
_[ Start new terminal window and run commands: ]_<br>
```
$ cd fog_sw/ros2_ws
$ source install/setup.bash
$ ros2 launch px4_mavlink_ctrl mavlink_ctrl.launch
```
### Enable FastRTPS bridge:
Start fastRTPS agent in ROS:<br>
_[ Start new terminal window and run commands: ]_<br>
```
$ cd fog_sw/ros2_ws/install/px4_ros_cmd/bin
$ ./micrortps_agent -t UDP
```

______________________________

# Examples
### RTPS stream - listen combined sensor messages from PX4
_[ Start new terminal window and run commands: ]_<br>
```
$ source fog_sw/ros2_ws/install/setup.bash
$ ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

### Mavlink cmds - send command to PX4
px4_mavlink_ctrl node subscribes /mavlinkcmd topic (type: std_msgs/msg/String) and sends mavlink messages to PX4 according to commands.<br>
Supported commands are:<br>

```
"takeoff"         : Drone takes off from the ground
"land"            : Drone lands to the ground
"start_mission"   : Drone start a pre-loaded mission
"pause_mission"   : Drone pauses currently ongoing mission
"resume_mission"  : Drone continues currently paused mission
"return_home"     : Drone immediately returns to the launch point
```

Start mission requires that there is already a mission loaded into the drone (e.g. by QGroundControl app)<br>
To test mission commands, send commands to topic, e.g:<br><br>
_[ Start new terminal window and run command: ]_<br>
```
$ ros2 topic pub -t 1 /mavlinkcmd std_msgs/msg/String "data: takeoff"
```
