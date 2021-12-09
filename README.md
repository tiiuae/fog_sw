# Drone HW setup instructions

asdfasfasfdas


Here are step by step instructions to setup ROS2 environment into Drone HW.

## Prerequisites

Recommended environment setup:
- Ubuntu 20.04 for host OS
- Docker installed for building fog-sw packages (see https://docs.docker.com/engine/install/ubuntu/ and https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user for Ubuntu installation instructions).
- python3 with invoke task execution package

## Build all submodules

### Setup invoke

Invoke can be used to easily execute different tasks for fog-sw repository, for example to build fog-sw. It is recommended to use python3 with invoke. See https://www.pyinvoke.org/installing.html for installation. Check https://docs.pyinvoke.org/en/stable/invoke.html#generating-a-completion-script to use shell completion with invoke. Invoke is used only convenience, similar commands can be executed manually from the command line.

### Clone this repository:

Clone the main fog-sw repository with normal git command, for example:
```
$ git clone https://github.com/tiiuae/fog_sw.git
```

Invoke command can be used after cloning the repository. See invoke task list with command:
```
$ cd fog_sw
$ invoke -l
```

Some of the invoke tasks contains additional arguments. Use "invoke <task> -h" command to see full task description.

Fog-sw repository contains both public and private submodules. You can stil build the public part of the fog-sw repository even though private repositories are missing from the workspace. Please contact repository admins, if you need access to private submodules.

Run following command to clone public fog-sw repositories:
```
$ invoke clone
```

### Build and generate debian packages

Public fog-sw full build can be triggered with command:
```
$ invoke build
```

Check "invoke build -h" for other build targets.

Build is executed in a docker environment, which contains all necessary tools and packages to build fog-sw modules. Initial build will take some time, because local build environment is created. Docker cache is used for following builds, which will reuse local build environment.

Debian packages are generated into fog_sw/packaging/deb_files folder.

You can also build debian packages directly in your host machine without using docker and invoke. Check build dependencies in 'update_dependencies.sh' and build scripts in 'packaging' directory for more details.
