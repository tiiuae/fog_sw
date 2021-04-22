#!/bin/bash

get_version() {
    pushd ../../systemd
    version=1.0.0~$(git describe --always --tags --dirty --match "[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/etc/systemd/system
	cp ../../systemd/packaging/debian/* ${build_dir}/DEBIAN/
	cp ../../systemd/system/* ${build_dir}/etc/systemd/system/
	mkdir -p ${build_dir}/opt/ros/foxy
	cp ../../systemd/setup_fog.sh ${build_dir}/opt/ros/foxy/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
	echo fog-sw-ros-systemd-services_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../fog-sw-ros-systemd-services_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
make_deb
