#!/bin/bash

get_version() {
	pushd ../../FastRTPS-1.8.2/
	version=1.0.0~$(git describe --always --tags --dirty --match "v[0-9]*.[0-9]*.[0-9]*")
	echo ${version}
	popd
}

build() {
	pushd ../../FastRTPS-1.8.2/
	mkdir build
	cd build
	mkdir install
	cmake .. || exit 1
	make install DESTDIR=$1 || exit 1
	popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$1
	mkdir ${build_dir}/DEBIAN
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
	echo fastrtpslib_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../fastrtpslib_${version}_amd64.deb

	echo "Done"
}

version=$(get_version)
tmp_dir=$(mktemp -d)
build ${tmp_dir}
make_deb ${tmp_dir}
rm -rf ${tmp_dir}
