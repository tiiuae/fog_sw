#!/bin/bash

set -e

if [ ! -e /opt/ros/galactic/setup.bash ]; then
  echo "ERROR: ROS2 environment cannot be found!"
  exit 1
fi
#RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/galactic/setup.bash
export PATH="/usr/local/go/bin:$PATH"

# build types: development build, release candidate, release
BUILD_TYPE=${1:-DEV}
RELEASE=${2:-5.0}

if [ "${BUILD_TYPE}" == "DEV" ];
then
  DEBIAN_REVISION="0~dirty"
elif [ "${BUILD_TYPE}" == "RC" ];
then
  DEBIAN_REVISION="rc${RELEASE}"
elif [ "${BUILD_TYPE}" == "REL" ];
then
  DEBIAN_REVISION="rel${RELEASE}"
else
  echo "ERROR: unsupported build type"
  exit 1
fi

set -uxo pipefail

# Get the path to this script.
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-}
SCRIPT_PATH=`dirname "$0"`
SCRIPT_PATH=`( cd "$SCRIPT_PATH" && pwd )`
DEBS_OUTPUT_DIR="deb_files"
DDEBS_OUTPUT_DIR="ddeb_files"

cd "${SCRIPT_PATH}"

if [ ! -e ${DEBS_OUTPUT_DIR} ]; then
  mkdir ${DEBS_OUTPUT_DIR}
fi

if [ ! -e ${DDEBS_OUTPUT_DIR} ]; then
  mkdir ${DDEBS_OUTPUT_DIR}
fi

if ! go version > /dev/null 2>&1; then
    echo "ERROR: missing Go!"
    echo "Install Go from https://go.dev/dl"
    echo "and ensure that the Go tool is in PATH."
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
    mv ../*.ddeb ${SCRIPT_PATH}/${DDEBS_OUTPUT_DIR}/
  fi
}

function _execute_build() {
  local version=$1
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro ${ROS_DISTRO} -i "${version}"
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
  ./package.sh "${DEBIAN_REVISION}"
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
  local version="${DEBIAN_REVISION}$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)"
  _execute_build "${version}"
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
  dpkg -s libsurvive || sudo dpkg -i ${SCRIPT_PATH}/${DEBS_OUTPUT_DIR}/libsurvive_*.deb
popd

# ROS packages
pushd ../ros2_ws/src/px4_msgs
  _make_ros_deb "px4-msgs"
  # Some of the following packages needs px4_msgs, so add it to the CMAKE paths
  if [ -z "${CMAKE_PREFIX_PATH}" ]; then
    export CMAKE_PREFIX_PATH=${PWD}/debian/ros-${ROS_DISTRO}-px4-msgs/opt/ros/${ROS_DISTRO}
  else
    export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-${ROS_DISTRO}-px4-msgs/opt/ros/${ROS_DISTRO}
  fi
popd

pushd ../ros2_ws/src/fog_msgs
  _make_ros_deb "fog_msgs"
  # Some of the following packages needs fog_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-${ROS_DISTRO}-fog-msgs/opt/ros/${ROS_DISTRO}
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

pushd ../ros2_ws/src/odometry2
  _make_ros_deb "odometry2"
popd

pushd ../ros2_ws/src/fog_bumper
  _make_ros_deb "fog-bumper"
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
