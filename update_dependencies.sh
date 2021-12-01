#!/bin/bash

set -euxo pipefail

ROS_DISTRO_="galactic"

sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu/ focal main restricted" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu/ focal-updates main restricted" >> /etc/apt/sources.list'

echo "Update Ubuntu repository"
sudo apt update

echo "Install or refresh dependencies"
sudo apt install -y \
    curl \
    wget \
    build-essential \
    dh-make debhelper \
    fakeroot \
    git-core \
    golang-1.16-go \
    libasio-dev \
    openjdk-11-jdk-headless \
    openssh-client \
    python3-bloom \
    python3-colcon-common-extensions \
    python3-pip \
    python3-future \
    python3-genmsg \
    ros-${ROS_DISTRO_}-ros-base \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstrtspserver-1.0-dev \
    nlohmann-json3-dev \
    ros-${ROS_DISTRO_}-geodesy \
    zlib1g-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libatlas-base-dev \
    gazebo11 \
    libgazebo11-dev \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev \
    libopencv-dev \
    libopencv-imgproc-dev \
    openjdk-11-jdk-headless \
    python3 \
    python3-empy \
    python3-jinja2 \
    python3-pip \
    python3-setuptools \
    python3-toml \
    python3-yaml \
    python3-packaging \
    python3-numpy \
    python3-genmsg \
    dh-python \
    batctl \
    alfred \
    ros-${ROS_DISTRO_}-octomap \
    ros-${ROS_DISTRO_}-octomap-msgs \
    ros-${ROS_DISTRO_}-laser-geometry \
    ros-${ROS_DISTRO_}-pcl-conversions \
    ros-${ROS_DISTRO_}-pcl-msgs \
    ros-${ROS_DISTRO_}-dynamic-edt-3d \
    ros-${ROS_DISTRO_}-gazebo-ros \
    ros-${ROS_DISTRO_}-rmw-dds-common \
    ros-${ROS_DISTRO_}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO_}-rmw-fastrtps-dynamic-cpp \
    ros-${ROS_DISTRO_}-rmw-fastrtps-shared-cpp \
    ros-${ROS_DISTRO_}-rmw-implementation \
    ros-${ROS_DISTRO_}-rosidl-typesupport-fastrtps-cpp \
    ros-${ROS_DISTRO_}-rosidl-typesupport-fastrtps-c \
    kernel-package \
    libncurses-dev \
    gawk \
    flex \
    bison \
    openssl \
    libssl-dev \
    libelf-dev \
    libudev-dev \
    libpci-dev \
    libiberty-dev \
    autoconf \
    linux-headers-generic \
    dh-exec \
    libdbus-1-dev \
    libpcsclite-dev \
    libnl-genl-3-dev \
    libreadline-dev \
    docbook-to-man

pip3 install --user pyros-genmsg

wget https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu20.04_amd64.deb
sudo dpkg -i mavsdk_0.34.0_ubuntu20.04_amd64.deb

# The following lines will make the substitution of ROS_DISTRO_ variable in rosdep_template.yaml.
rm -rf rosdep.yaml tmp.yaml
( echo "cat <<EOF >rosdep.yaml";
  cat rosdep_template.yaml;
  echo "EOF";
) >tmp.yaml
. tmp.yaml
cat rosdep.yaml

echo "--- Generating /etc/ros/rosdep/sources.list.d/50-fogsw.list (as su)"
sudo sh -c 'mkdir -p /etc/ros/rosdep/sources.list.d'
sudo sh -c 'echo "yaml file://${PWD}/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-fogsw.list'

if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
	echo "--- Initialize rosdep"
	sudo rosdep init
fi

echo "--- Updating rosdep"
rosdep update

sudo apt-get upgrade -y

exit 0
