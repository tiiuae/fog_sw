#!/bin/bash

get_version() {
    version=3.2rc~$(git log --date=format:%Y%m%d --pretty=git%cd.%h -n 1)
    echo ${version}
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN

	cp packaging/debian/* ${build_dir}/DEBIAN/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
	echo fog-sw-full_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../fog-sw-full_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
make_deb
