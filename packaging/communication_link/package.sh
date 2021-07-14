#!/bin/bash

pushd ../../ros2_ws/src/communication_link > /dev/null
export CGO_CFLAGS="-I$(realpath ../../install/px4_msgs/include/)"
export CGO_LDFLAGS="-L$(realpath ../../install/px4_msgs/lib/)"
mkdir -p communicationlink/packaging/common
cp -fR scripts/* communicationlink/packaging/common/
cd communicationlink
params="-m $(realpath .) -v 2.0.0 -c $(git rev-parse HEAD) -g $(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1) -b dirty"
./packaging/common/package.sh $params
popd

exit 0
