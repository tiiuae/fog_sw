#!/bin/bash

set -e

if [ ! -e /opt/ros/foxy/setup.bash ]; then
  echo "ERROR: ROS2 environment cannot be found!"
  exit 1
fi

source /opt/ros/foxy/setup.bash

# Get the path to this script.
SCRIPT_PATH=`dirname "$0"`
SCRIPT_PATH=`( cd "$SCRIPT_PATH" && pwd )`
DEBS_OUTPUT_DIR="deb_files"

if [ ! -e ${DEBS_OUTPUT_DIR} ]; then
  mkdir ${DEBS_OUTPUT_DIR}
fi

if [ ! -e /usr/lib/go-1.16/bin/ ] ; then
    echo "ERROR: missing golang v1.16!"
    echo "Install golang v1.16 package: golang-1.16-go."
    exit 1
fi

# Speed up builds.
# In addition to the following environmental variable,
# --parallel flag is needed in "fakeroot debian/rules binary" call.
export DEB_BUILD_OPTIONS="parallel=`nproc`"

function _move_debs() {
  if ls ../*.deb 1> /dev/null 2>&1; then
    mv ../*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/
  fi
  if ls ../*.ddeb 1> /dev/null 2>&1; then
    mv ../*.ddeb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/
  fi
}

function _execute_build() {
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy -i "${GIT_VER}"
  sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules
  fakeroot debian/rules clean
  fakeroot debian/rules "binary --parallel"
}

function _make_ros_deb() {
  echo "Creating deb package ${1} ..."
  GIT_VER=0~dirty$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
  build_dir=$(mktemp -d)
  cp -r . ${build_dir}/package
  pushd ${build_dir}/package > /dev/null
  _execute_build
  _move_debs
  popd > /dev/null
  rm -rf ${build_dir}
  echo "Done."
}

# Non-ROS packages
pushd agent_protocol_splitter
  ./package.sh
popd
mv ./agent-protocol-splitter*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd fogsw_configs
  ./package.sh
popd
mv ./fogsw-configs*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd mavlink-router
  ./package.sh
popd
mv ./mavlink-router*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd mission-data-recorder
  ./package.sh
popd
mv ../mission-data-recorder/mission-data-recorder*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd systemd
  ./package.sh
popd
mv ./fog-sw-systemd*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

# TODO: fix the package_wpa.sh under wpa repo in order to work in local builds.
# Local build does not have build directory so package_wpa.sh fails.
#pushd wpasupplicant
#  ./package.sh
#popd
#mv ./wpasupplicant*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

# Not needed yet.
#pushd ../fogsw_secure_os
#./package.sh
#popd
#mv ../fogsw-secure-os*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

# ROS packages

pushd ../ros2_ws/src/px4_msgs
  echo "Creating deb package px4-msgs ..."
  GIT_VER=0~dirty$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
  _execute_build
  _move_debs
  rm -rf obj-x86_64-linux-gnu
  # Some of the following packages needs px4_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy
  export px4_msgs_DIR=${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy/share/px4_msgs/cmake
  echo "Done."
popd


pushd ../ros2_ws/src/fog_msgs
  echo "Creating deb package fog-msgs ..."
  GIT_VER=0~dirty$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
  _execute_build
  _move_debs
  rm -rf obj-x86_64-linux-gnu
  # Some of the following packages needs fog_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy
  export fog_msgs_DIR=${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy/share/fog_msgs/cmake
  echo "Done."
popd

pushd communication_link
  ./package.sh
popd
mv ../ros2_ws/src/communication_link/communication-link*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd mission-engine
  ./package.sh
popd
mv ../ros2_ws/src/communication_link/mission-engine*.deb ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/

pushd ../ros2_ws/src/mesh_com/
  echo "Creating deb package mesh-com ..."
  GIT_VER=0~dirty$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
  build_dir=$(mktemp -d)
  cp -r . ${build_dir}/package
  pushd ${build_dir}/package/modules/mesh_com > /dev/null
  _execute_build
  _move_debs
  popd > /dev/null
  rm -rf ${build_dir}
  echo "Done."
popd

pushd ../ros2_ws/src/depthai_ctrl
  _make_ros_deb "depthai-ctrl"
popd

pushd ../ros2_ws/src/px4_ros_com
  _make_ros_deb "px4-ros-com"
popd

pushd ../ros2_ws/src/indoor_pos
  _make_ros_deb "indoor-pos"
popd

pushd ../ros2_ws/src/fog_gazebo_resources
  _make_ros_deb "fog-gazebo-resources"
popd

pushd ../ros2_ws/src/rplidar_ros2
  _make_ros_deb "rplidar-ros2"
popd

pushd ../ros2_ws/src/control_interface
  _make_ros_deb "control-interface"
popd

pushd ../ros2_ws/src/octomap_server2
  _make_ros_deb "octomap-server2"
popd

pushd ../ros2_ws/src/navigation
  _make_ros_deb "navigation"
popd

pushd ../ros2_ws/src/mocap_pose
  _make_ros_deb "mocap-pose"
popd

exit 0
