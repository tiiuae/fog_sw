#!/bin/bash

set -e

if [ ! -e /opt/ros/galactic/setup.bash ]; then
  echo "ERROR: ROS2 environment cannot be found!"
  exit 1
fi

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

cd "${SCRIPT_PATH}"

if [ ! -e ${DEBS_OUTPUT_DIR} ]; then
  mkdir ${DEBS_OUTPUT_DIR}
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

pushd agent_protocol_splitter
  _make_deb agent_protocol_splitter
popd

pushd fogsw_configs
  _make_deb fogsw_configs
popd

pushd mavlink-router
  _make_deb mavlink-router
popd

pushd systemd
  _make_deb systemd
popd

pushd wpasupplicant
  _make_deb wpasupplicant
popd

pushd wifi-firmware
  _make_deb wifi-firmware
popd
