#!/bin/bash

get_version() {
    pushd ../../ros2_ws/src/communication_link
    version=1.0.0~$(git describe --always --tags --dirty --match "[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

build() {
	pushd ../../ros2_ws/src/communication_link
	export GOPRIVATE=github.com/ssrc-tii
	source /opt/ros/foxy/setup.bash
	go build
	cp communication_link ../../../packaging/communication_link/ && go clean
        popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/bin/
	mkdir ${build_dir}/private
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp communication_link ${build_dir}/usr/bin/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
        echo communication-link_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../communication-link_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
build
make_deb
