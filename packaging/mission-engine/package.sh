#!/bin/bash

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

cd "${THIS_DIR}"/../../ros2_ws/src/communication_link

export CGO_CFLAGS=
export CGO_LDFLAGS=
# build depedency to px4_msgs
CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/include/)"
CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/lib/)"
# build depedency to fog_msgs
CGO_CFLAGS="${CGO_CFLAGS} -I$(realpath ../fog_msgs/debian/ros-galactic-fog-msgs/opt/ros/galactic/include/)"
CGO_LDFLAGS="${CGO_LDFLAGS} -L$(realpath ../fog_msgs/debian/ros-galactic-fog-msgs/opt/ros/galactic/lib/)"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/
cp ./missionengine/packaging/debian/* "${build_dir}"/DEBIAN/

pushd ./missionengine/cmd
go build -o mission-engine
cp -f mission-engine "${build_dir}"/usr/bin/
popd

exit 0
