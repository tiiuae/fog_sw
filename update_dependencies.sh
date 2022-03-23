#!/bin/bash

set -euxo pipefail

ROS_DISTRO_="galactic"
GO_VERSION=1.17.5

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
    ros-${ROS_DISTRO_}-rmw-implementation \
    ros-${ROS_DISTRO_}-osrf-testing-tools-cpp \
    ros-${ROS_DISTRO_}-test-msgs \
    ros-${ROS_DISTRO_}-performance-test-fixture \
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
    docbook-to-man \
    libasio-dev \
    libengine-pkcs11-openssl \
    libp11-dev \
    libssl-dev \
    libtinyxml2-dev

curl -L https://go.dev/dl/go${GO_VERSION}.linux-amd64.tar.gz \
| tar -xzC /usr/local

pip3 install --user pyros-genmsg

curl -LO https://ssrc.jfrog.io/artifactory/ssrc-debian-release-remote/mavsdk_0.42.0_ubuntu20.04_amd64.deb
sudo dpkg -i mavsdk_0.42.0_ubuntu20.04_amd64.deb

# Packages needed in SROS + PKCS#11.
SROS_PKCS11_PKGS=(
  ros-${ROS_DISTRO_}-foonathan-memory-vendor_1.1.0-4~git20220310.bbb8a5c_amd64.deb
  ros-${ROS_DISTRO_}-fastcdr_1.0.20-5~git20220310.f65f034_amd64.deb
  ros-${ROS_DISTRO_}-fastrtps_2.5.0-7~git20220310.4ca1f95_amd64.deb
  ros-${ROS_DISTRO_}-fastrtps-cmake-module_1.2.1-6~git20220310.67ed436_amd64.deb
  ros-${ROS_DISTRO_}-rmw-fastrtps-shared-cpp_5.0.0-7~git20220310.8684e20_amd64.deb
  ros-${ROS_DISTRO_}-rosidl-typesupport-fastrtps-cpp_1.2.1-6~git20220310.67ed436_amd64.deb
  ros-${ROS_DISTRO_}-rosidl-typesupport-fastrtps-c_1.2.1-6~git20220310.67ed436_amd64.deb
  ros-${ROS_DISTRO_}-px4-msgs_3.0.0-15~git20220104.c12fcdf_amd64.deb
  ros-${ROS_DISTRO_}-fog-msgs_0.0.8-42~git20220104.1d2cf3f_amd64.deb
)
for PKG in "${SROS_PKCS11_PKGS[@]}"; do
  curl -LO https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/$PKG
  sudo dpkg -i $PKG
done

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
