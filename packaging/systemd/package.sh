#!/bin/bash

get_version() {
    version=1.0.0~$(git describe --always --tags --match "[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/etc/systemd/system
	mkdir -p ${build_dir}/etc/udev/rules.d
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp system/* ${build_dir}/etc/systemd/system/
	cp udev-rules/* ${build_dir}/etc/udev/rules.d/
	mkdir -p ${build_dir}/opt/ros/foxy
	cp setup_fog.sh ${build_dir}/opt/ros/foxy/

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
