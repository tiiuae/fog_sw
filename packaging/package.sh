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
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd ../ros2_ws/src/px4_msgs
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
# The following px4_ros_com needs px4_msgs, which was just compiled, so add it to the CMAKE paths
export CMAKE_PREFIX_PATH=${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy
popd

pushd ../ros2_ws/src/px4_ros_com
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd ../ros2_ws/src/indoor_pos
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd ../simulation/src/sim_mesh_ctrl
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/
popd

pushd communication_link
./package.sh
popd
