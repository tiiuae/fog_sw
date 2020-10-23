#!/bin/bash

pushd agent_protocol_splitter
./package.sh
popd

pushd systemd
./package.sh
popd

pushd mavlink-router
./package.sh
popd

pushd ../ros2_ws/src/px4_mavlink_ctrl
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd ../ros2_ws/src/px4_ros_com
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd ../ros2_ws/src/px4_msgs
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd communication_link
./package.sh
popd
