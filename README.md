# Drone HW setup instructions

Here are step by step instructions to setup ROS2 environment into Drone HW.

## ROS2 installation

Install ROS2 following instructions here:<br>
https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

Install dependencies to build fog_sw:<br>
```
$ sudo apt update
$ sudo apt install \
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
    ros-foxy-ros-base \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    nlohmann-json3-dev \
    ros-foxy-geodesy \
    zlib1g-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libatlas-base-dev \
    ros-foxy-gazebo-ros
$ pip3 install --user pyros-genmsg
$ wget https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu20.04_amd64.deb
$ sudo dpkg -i mavsdk_0.34.0_ubuntu20.04_amd64.deb
```
Add ROS2 script start into startup script (e.g. ~/.bashrc)<br>
`$ source /opt/ros/foxy/setup.bash`


## PX4 & gazebo dependencies installation
```
$ sudo apt update
$ sudo apt install \
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
    python3-genmsg

```
## Batman mesh dependencies installation
```
$ sudo apt update
$ sudo apt install \
    dh-python \
    batctl \
    alfred

```
## Build all submodules

### Clone this repository:
```
$ git clone https://github.com/tiiuae/fog_sw.git
$ cd fog_sw
$ git submodule update --init --recursive
$ cd ..
```

### Install dependencies
The update_dependencies.sh script will ask for root credentials (sudo).
```
$ cd fog_sw
$ chmod +x update_dependencies.sh
$ ./update_dependencies.sh
$ popd
```

### Build and generate debian packages

Generate deb files:
```
$ cd fog_sw/packaging
$ ./package_all.sh
```
Debian packages are generated into fog_sw/packaging/deb_files folder.
