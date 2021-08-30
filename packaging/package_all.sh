#!/bin/bash

set -e

if [ ! -e /opt/ros/foxy/setup.bash ]; then
  echo "ERROR: ROS2 environment cannot be found!"
  exit 1
fi

source /opt/ros/foxy/setup.bash

set -uxo pipefail

# Get the path to this script.
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-}
SCRIPT_PATH=`dirname "$0"`
SCRIPT_PATH=`( cd "$SCRIPT_PATH" && pwd )`
DEBS_OUTPUT_DIR="deb_files"

cd "${SCRIPT_PATH}"

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
}

function _execute_build() {
#  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy --place-template-files
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy -i "${GIT_VER}"
  sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules
  fakeroot debian/rules clean
  fakeroot debian/rules "binary --parallel"
}

function _make_deb() {
  local pkg=$1
  cat <<- EOF
  *********************************************************
  Building deb package ${pkg} 
  *********************************************************
EOF
  ./package.sh
  _move_debs
  echo "Done."
}

function _make_ros_deb() {
  local pkg=$1
  cat <<- EOF
  *********************************************************
  Building ROS deb package ${pkg} 
  *********************************************************
EOF
  GIT_VER=0~dirty$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
  _execute_build
  _move_debs
  echo "Done."
}

# Non-ROS packages
pushd ../tools/Fast-DDS-Gen
  cat <<- EOF
  *********************************************************
  Fast-DDS-Gen build 
  *********************************************************
EOF
  ./gradlew build
  export PATH=$PATH:$PWD/scripts
popd

pushd libsurvive
  _make_deb libsurvive
  sudo dpkg -i ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/libsurvive_*.deb
popd

pushd rtl8812au
  _make_deb rtl8812au
popd

pushd agent_protocol_splitter
  _make_deb agent_protocol_splitter
popd

pushd fogsw_configs
  _make_deb fogsw_configs
popd

pushd mavlink-router
  _make_deb mavlink-router
popd

pushd mission-data-recorder
  _make_deb mission-data-recorder
popd

pushd systemd
  _make_deb systemd
popd

pushd fogsw_kernel_config
  _make_deb fogsw_kernel_config
popd

pushd wpasupplicant
  _make_deb wpasupplicant
popd

# ROS packages
pushd ../ros2_ws/src/px4_msgs
  _make_ros_deb "px4-msgs"
  # Some of the following packages needs px4_msgs, so add it to the CMAKE paths
  if [ -z "${CMAKE_PREFIX_PATH}" ]; then
    export CMAKE_PREFIX_PATH=${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy
  else
    export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy
  fi
popd

pushd ../ros2_ws/src/fog_msgs
  _make_ros_deb "fog_msgs"
  # Some of the following packages needs fog_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy
popd

pushd ../ros2_ws/src/px4_ros_com
  _make_ros_deb "px4-ros-com"
popd

pushd communication_link
  _make_deb communication_link
popd

pushd mission-engine
  _make_deb mission-engine
popd

pushd ../ros2_ws/src/mesh_com/modules/mesh_com
  _make_ros_deb "mesh_com"
popd

pushd ../ros2_ws/src/depthai_ctrl
  _make_ros_deb "depthai-ctrl"
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
