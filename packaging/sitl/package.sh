#!/bin/bash

source /opt/ros/foxy/setup.sh

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"
fw_dir="${THIS_DIR}/../../px4-firmware"
dest_dir="${THIS_DIR}/../px4_files"

fastddsgen="${THIS_DIR}/../../tools/Fast-DDS-Gen"

if [ ! -e "${fastddsgen}/share/fastddsgen/java/fastddsgen.jar" ];
then
	echo "ERROR: ${fastddsgen}/fastddsgen.jar not found!"
	exit 1
fi

export PATH=$PATH:"${fastddsgen}/scripts"

mkdir -p "${dest_dir}"

pushd ${fw_dir}
version=$(git describe --always --tags --dirty | sed 's/^v//')
make clean
DONT_RUN=1 make px4_sitl_rtps gazebo_ssrc_fog_x

# sitl
tmp=$(mktemp -d)
pushd "${tmp}"
mkdir -p px4_sitl/build/px4_sitl_rtps
cp -r "${fw_dir}/build/px4_sitl_rtps/bin" px4_sitl/build/px4_sitl_rtps/bin
cp -r "${fw_dir}/build/px4_sitl_rtps/etc" px4_sitl/build/px4_sitl_rtps/etc
tar -zcf "px4_sitl_build-${version}.tar.gz" px4_sitl
mv "px4_sitl_build-${version}.tar.gz" "${dest_dir}"
popd # tmp
rm -rf "${tmp}"

# gazebo-data
tmp=$(mktemp -d)
pushd "${tmp}"
mkdir -p px4_gazebo_data/plugins \
         px4_gazebo_data/models \
         px4_gazebo_data/worlds \
         px4_gazebo_data/scripts
cp -r "${fw_dir}"/build/px4_sitl_rtps/build_gazebo/*.so      px4_gazebo_data/plugins
cp -r "${fw_dir}"/Tools/sitl_gazebo/models/asphalt_plane     px4_gazebo_data/models/asphalt_plane
cp -r "${fw_dir}"/Tools/sitl_gazebo/models/ground_plane      px4_gazebo_data/models/ground_plane
cp -r "${fw_dir}"/Tools/sitl_gazebo/models/sun               px4_gazebo_data/models/sun
cp -r "${fw_dir}"/Tools/sitl_gazebo/models/ssrc_fog_x        px4_gazebo_data/models/ssrc_fog_x
cp -r "${fw_dir}"/Tools/sitl_gazebo/worlds/empty.world       px4_gazebo_data/worlds/
cp -r "${fw_dir}"/Tools/sitl_gazebo/worlds/empty_ssrc.world  px4_gazebo_data/worlds/
cp -r "${fw_dir}"/Tools/sitl_gazebo/scripts/jinja_gen.py     px4_gazebo_data/scripts/
tar -zcf "px4_gazebo_data-${version}.tar.gz" px4_gazebo_data
mv "px4_gazebo_data-${version}.tar.gz" "${dest_dir}"
popd # tmp
rm -rf "${tmp}"

popd # fw_dir
