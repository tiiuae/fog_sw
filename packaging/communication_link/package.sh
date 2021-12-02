#!/bin/bash

THIS_DIR="$(dirname "$(readlink -f "$0")")"

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

cd "${THIS_DIR}"/../../ros2_ws/src/communication_link

# build depedency to px4_msgs
export CGO_CFLAGS=
export CGO_LDFLAGS=
CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/include/)"
CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/lib/)"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/

pushd communicationlink
cp ./packaging/debian/* "${build_dir}"/DEBIAN/
go build -o communication_link
cp -f communication_link ${build_dir}/usr/bin/
popd

exit 0
