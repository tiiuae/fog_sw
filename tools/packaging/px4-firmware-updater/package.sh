#!/bin/bash

get_version() {
	version=$(cat debian/control | grep Version | cut -d' ' -f2)
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN/
	mkdir -p ${build_dir}/opt/px4fwupdater/
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp px4_fmu-v5_ssrc.px4 ${build_dir}/opt/px4fwupdater/
	cp px_uploader.py ${build_dir}/opt/px4fwupdater/

	cat ${build_dir}/DEBIAN/control
	echo px4fwupdater_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../px4fwupdater_${version}_amd64.deb
	rm -rf ${build_dir}

	echo "Done"
}

get_version
make_deb
