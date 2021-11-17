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
SIGNING_TOOL=Tools/cryptotools.py make px4_fmu-v5_ssrc
SIGNING_TOOL=Tools/cryptotools.py make px4_fmu-v5x_ssrc
cp build/px4_fmu-v5_ssrc/px4_fmu-v5_ssrc.px4 "${dest_dir}/px4_fmu-v5_ssrc-${version}.px4"
cp build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4 "${dest_dir}/px4_fmu-v5x_ssrc-${version}.px4"
popd
