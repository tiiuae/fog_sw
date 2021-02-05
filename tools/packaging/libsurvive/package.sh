#!/bin/bash

get_version() {
    pushd ../../libsurvive/
    version=1.0.0~$(git describe --always --tags --dirty --match "v[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/lib/
	mkdir -p ${build_dir}/usr/include/libsurvive/
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/

	pushd ../../libsurvive/
	cmake -DCMAKE_INSTALL_PREFIX=${build_dir}/usr . && make all install
	popd

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
	echo libsurvive_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../libsurvive_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
make_deb
