#!/bin/bash

get_version() {
    pushd ../../agent_protocol_splitter
    version=1.0.0$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
    echo ${version}
    popd
}

build() {
	mkdir -p ../../agent_protocol_splitter/build
	pushd ../../agent_protocol_splitter/build
	cmake ..
	make || exit 1
	cp protocol_splitter ../../packaging/agent_protocol_splitter/ && make clean
	rm -dr ../../agent_protocol_splitter/build
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
