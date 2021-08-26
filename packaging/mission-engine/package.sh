#!/bin/bash

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

pushd ../../ros2_ws/src/communication_link > /dev/null
export CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/include/) -I$(realpath ../fog_msgs/debian/ros-foxy-fog-msgs/opt/ros/foxy/include/)"
export CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/lib/) -L$(realpath ../fog_msgs/debian/ros-foxy-fog-msgs/opt/ros/foxy/lib/)"
mkdir -p missionengine/packaging/common
cp -fR scripts/* missionengine/packaging/common/
cd missionengine
params="-m $(realpath .) -v 2.0.0 -c $(git rev-parse HEAD) -g $(git log --date=format:%Y%m%d --pretty=~dirty~git%cd.%h -n 1) -b 0"
./packaging/common/package.sh $params
popd

mv ../../ros2_ws/src/communication_link/*.deb ../

exit 0
