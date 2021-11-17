#!/bin/bash

source /opt/ros/foxy/setup.sh

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"
fw_dir="${THIS_DIR}/../../px4-firmware"
px4_files="${THIS_DIR}/../px4_files"
dest_dir="${THIS_DIR}/../deb_files"

pushd "${fw_dir}"
version=$(git describe --always --tags --dirty | sed 's/^v//')
popd

packaging_dir=$(mktemp -d)

mkdir -p "${packaging_dir}"/DEBIAN \
         "${packaging_dir}"/opt/px4fwupdater

cp "${THIS_DIR}"/debian/control   "${packaging_dir}"/DEBIAN/
cp "${THIS_DIR}"/debian/postinst  "${packaging_dir}"/DEBIAN/
cp "${THIS_DIR}"/debian/prerm     "${packaging_dir}"/DEBIAN/
cp "${THIS_DIR}"/px4_update_fw.sh "${packaging_dir}"/opt/px4fwupdater/px4_update_fw.sh
cp "${THIS_DIR}"/detect_ttyS.sh   "${packaging_dir}"/opt/px4fwupdater/detect_ttyS.sh

cp "${fw_dir}"/Tools/px_uploader.py "${packaging_dir}"/opt/px4fwupdater/

cp "${px4_files}"/px4_fmu-v5_ssrc-*.px4          "${packaging_dir}"/opt/px4fwupdater/px4_fmu-v5_ssrc.px4
cp "${px4_files}"/px4_fmu-v5x_ssrc-*.px4         "${packaging_dir}"/opt/px4fwupdater/px4_fmu-v5x_ssrc.px4
cp "${px4_files}"/ssrc_saluki-v1_default-*.px4   "${packaging_dir}"/opt/px4fwupdater/ssrc_saluki-v1_default.px4

sed -i "s/Version:.*/&${version}/" ${packaging_dir}/DEBIAN/control
cat ${packaging_dir}/DEBIAN/control

echo px4fwupdater_${version}_amd64.deb
fakeroot dpkg-deb --build ${packaging_dir} ${dest_dir}/px4fwupdater_${version}_amd64.deb

rm -rf ${packaging_dir}
