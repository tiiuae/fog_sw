#!/bin/bash

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"

cd ../../rtl8812au

version=1.0.0$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
apt-get source linux-hwe-5.8-source-5.8.0
LINUX_SRC="${PWD}/linux-hwe-5.8-5.8.0"


pushd "${LINUX_SRC}"
    cp "${THIS_DIR}/config-5.8.0-55-generic" .config
    make modules_prepare
popd

# Build the driver here.
make KSRC="${LINUX_SRC}" modules

# make debian package
build_dir=$(mktemp -d)

# Copy debian files from mod_specific directory
cp -r "${THIS_DIR}"/debian "${build_dir}"/DEBIAN

mkdir -p "${build_dir}"/opt/rtl8812au_kmod
cp ./8812au.ko "${build_dir}"/opt/rtl8812au_kmod/

mkdir -p "${build_dir}"/etc/modules-load.d
cp "${THIS_DIR}"/rtl8812au.conf "${build_dir}"/etc/modules-load.d/rtl8812au.conf

sed -i "s/VERSION/${version}/" "${build_dir}"/DEBIAN/control
cat "${build_dir}"/DEBIAN/control
fakeroot dpkg-deb --build "${build_dir}" "${THIS_DIR}"/../deb_files/rtl8812au_"${version}"_amd64.deb

rm -rf ${build_dir}
