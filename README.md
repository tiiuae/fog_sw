# Drone HW setup instructions

Here are step by step instructions to setup ROS2 environment into Drone HW.

## Prerequisites

Recommended environment setup:
- Ubuntu 20.04 for host OS
- Docker installed for building fog-sw packages (see https://docs.docker.com/engine/install/ubuntu/ and https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user for Ubuntu installation instructions).

## Build all submodules

### Clone this repository:

Simply run following command to clone this repository:
```
$ git clone https://github.com/tiiuae/fog_sw.git --recursive
```
You might receive "permission denied" errors during the git clone for private submodules. However, you can stil build the public part of the fog-sw repository even though private repositories are missing. Please contact repository admins, if you need access to private submodules.

### Build and generate debian packages

Generate deb files:
```
$ cd fog_sw
$ ./build.sh
```
Build is executed in a docker environment, which contains all required tools and packages to build fog-sw modules. Initial build will take some time, because the fog-sw build environment is created to local docker image. Following builds can reuse the existing docker build environment, which makes the build much faster.

Debian packages are generated into fog_sw/packaging/deb_files folder.

You can also build debian packages directly in your host machine without using docker build environment. First check build dependencies in 'update_dependencies.sh' script and then run 'packaging/package_all.sh' to run local fog-sw full build.
