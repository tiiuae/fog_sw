#!/bin/bash

# creates .deb, but keeps temporal dir and sources it
# use it for msgs or other shared packages
function make_ros_deb() {
  	echo "Creating deb package ${1} ..."
  	build_dir=$(mktemp -d)
  	cp -r . ${build_dir}/package
    pushd ${build_dir}/package > /dev/null
    bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
    fakeroot debian/rules binary
    popd > /dev/null
    mv ${build_dir}/*.*deb ${MY_PATH}/deb_files/
    mv ${build_dir}/*.*ddeb ${MY_PATH}/deb_files/
    rm -rf ${build_dir}
    export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-${1}/opt/ros/foxy
  	echo "Done."
}

# Non-ROS packages
pushd agent_protocol_splitter
./package.sh
popd

pushd systemd
./package.sh
popd

pushd mavlink-router
./package.sh
popd

pushd communication_link
./package.sh
popd

pushd ../fogsw_secure_os
./package.sh
popd
mv ../fogsw-secure-os*.deb .


# ROS packages

pushd ../ros2_ws/src/px4_msgs
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
  fakeroot debian/rules binary
  mv ../*.*deb ../../../packaging/deb_files/
  rm -rf obj-x86_64-linux-gnu
  # Some of the following packages needs px4_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy
  export px4_msgs_DIR=${PWD}/debian/ros-foxy-px4-msgs/opt/ros/foxy/share/px4_msgs/cmake
popd


pushd ../ros2_ws/src/fog_msgs
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
  fakeroot debian/rules binary
  mv ../*.*deb ../../../packaging/deb_files/
  rm -rf obj-x86_64-linux-gnu
  # Some of the following packages needs fog_msgs, so add it to the CMAKE paths
  export CMAKE_PREFIX_PATH=${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy
  export fog_msgs_DIR=${PWD}/debian/ros-foxy-fog-msgs/opt/ros/foxy/share/fog_msgs/cmake
popd

pushd ../ros2_ws/src/px4_mavlink_ctrl
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
  sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules
  fakeroot debian/rules binary &&
  mv ../*.*deb ../../../packaging/deb_files/
  rm -rf debian obj-x86_64-linux-gnu
popd

pushd ../ros2_ws/src/depthai_ctrl
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
  sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules
  fakeroot debian/rules binary
  mv ../*.*deb ../../../packaging/deb_files
  rm -rf obj-x86_64-linux-gnu
popd

pushd ../ros2_ws/src/control_interface > /dev/null
  bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
  sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules
  fakeroot debian/rules binary
  mv ../*.*deb ../../../packaging/deb_files/
  rm -rf debian obj-x86_64-linux-gnu
popd > /dev/null

pushd ../ros2_ws/src/mesh_com/modules/mesh_com
  make_ros_deb "mesh-com"
popd

pushd ../ros2_ws/src/px4_ros_com
  make_ros_deb "px4-ros-com"
popd

pushd ../ros2_ws/src/indoor_pos
  make_ros_deb "px4-ros-com"
popd

pushd ../ros2_ws/src/fog_gazebo_resources > /dev/null
  make_ros_deb "fog-gazebo-resources"
popd > /dev/null

pushd ../ros2_ws/src/rplidar_ros2 > /dev/null
  make_ros_deb "rplidar-ros2"
popd > /dev/null

pushd ../ros2_ws/src/octomap_server2 > /dev/null
  make_ros_deb "octomap_server2"
popd > /dev/null

pushd ../ros2_ws/src/navigation > /dev/null
  make_ros_deb "navigation"
popd > /dev/null

exit 0
