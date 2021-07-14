#!/bin/bash

pushd ../../ros2_ws/src/communication_link > /dev/null
export CGO_CFLAGS="-I$(realpath ../../install/px4_msgs/include/) -I$(realpath ../../install/fog_msgs/include/)"
export CGO_LDFLAGS="-L$(realpath ../../install/px4_msgs/lib/) -L$(realpath ../../install/fog_msgs/lib/)"
mkdir -p missionengine/packaging/common
cp -fR scripts/* missionengine/packaging/common/
cd missionengine
params="-m $(realpath .) -v 2.0.0 -c $(git rev-parse HEAD) -g $(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1) -b dirty"
./packaging/common/package.sh $params
popd

exit 0
