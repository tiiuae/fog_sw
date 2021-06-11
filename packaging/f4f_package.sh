#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

options=$(getopt -l "create,clean" -o "" -a -- "$@")

eval set -- "$options"

while true
do
  case $1 in
    --create)
      CREATE=true
      shift
      ;;
    --clean)
      CLEAN=true
      shift
      ;;
    --)
      shift
      break
      ;;
  esac
done

[ -z "$CREATE" ] && [ -z "$CLEAN" ] && echo "$0: Choose --create, --clean, or both" && exit 1

cd "$MY_PATH"

## | ---------  cleaning after creation of debian packages  --------- |

function clean_package() {

  if [ -n "$1" ]; then
    pushd ../ros2_ws/src/$1 > /dev/null
  fi
  echo "Cleaned \"`basename "$PWD"`\""
  if [ -d debian ]; then
    rm -rf debian  
  fi
  if [ -d obj-x86_64-linux-gnu ]; then
    rm -rf debian obj-x86_64-linux-gnu 
  fi
  if [ -n "$1" ]; then
    popd > /dev/null
  fi
}

if [ -n "$CLEAN" ];
then

  echo "$0: Removing helper archive folders"
  clean_package fog_msgs
  clean_package fog_gazebo_resources
  clean_package octomap_server2
  clean_package control_interface
  clean_package navigation
  clean_package control_interface

fi

## | ---------  create debian packages for these pkg  --------- |

if [ -n "$CREATE" ];
then

  function move_archives() {
    if [ -f ../*.deb ]; then
      mv ../*.deb ../../../packaging/deb_files/
    else
      echo -e "\e[01;31mDeb file doesn't exist. Clean the helper archive files -> Call \"f4f_package.sh --clean\"\e[0m"
      exit 1
    fi
    if [ -f ../*.ddeb ]; then
      mv ../*.ddeb ../../../packaging/deb_files/
    fi
  }

  pushd ../ros2_ws/src/fog_msgs > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary 
  move_archives
  export CMAKE_PREFIX_PATH=${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy
  export fog_msgs_DIR=${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy/share/fog_msgs/cmake
  popd > /dev/null

  # package that depends on mavsdk needs little hack
  pushd ../ros2_ws/src/control_interface > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules && fakeroot debian/rules binary
  move_archives
  popd > /dev/null

  pushd ../ros2_ws/src/fog_gazebo_resources > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary 
  move_archives
  popd > /dev/null

  # we need to keep `debian` folder untouched, so .deb build happens in temporary dir
  pushd ../ros2_ws/src/rplidar_ros2 > /dev/null
  	echo "Creating deb package ${1} ..."
  	build_dir=$(mktemp -d)
  	cp -r . ${build_dir}/package
    pushd ${build_dir}/package > /dev/null
    bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
    fakeroot debian/rules binary
    popd > /dev/null
    mv ${build_dir}/*.*deb ${MY_PATH}/deb_files/
    rm -rf ${build_dir}
  	echo "Done."
  popd > /dev/null

  pushd ../ros2_ws/src/octomap_server2 > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary
  move_archives
  popd > /dev/null

  pushd ../ros2_ws/src/navigation > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary 
  move_archives
  popd > /dev/null

  pushd ../ros2_ws/src/octomap_server2 > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary
  move_archives
  popd > /dev/null



fi
