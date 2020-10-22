#!/bin/bash

get_version() {
    pushd ../../agent_protocol_splitter
    version=1.0.0~$(git describe --always --tags --dirty --match "[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

build() {
	pushd ../../agent_protocol_splitter/src
	cmake ..
	make || exit 1
	cp protocol_splitter ../../packaging/agent_protocol_splitter/ && make clean
        popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/bin/
	mkdir -p ${build_dir}/etc/systemd/system
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp protocol_splitter ${build_dir}/usr/bin/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
        echo agent_protocol_splitter_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../agent-protocol-splitter_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
build
make_deb
