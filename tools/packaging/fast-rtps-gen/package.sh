#!/bin/bash

get_version() {
    pushd ../../Fast-RTPS-Gen/
    version=1.0.0~$(git describe --always --tags --dirty --match "v[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

build() {
	pushd ../../Fast-RTPS-Gen/
	./gradlew build || exit 1
	cp build/libs/fastrtpsgen.jar ../packaging/fast-rtps-gen/
	cp scripts/fastrtpsgen ../packaging/fast-rtps-gen/
	popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/bin/
	mkdir -p ${build_dir}/usr/share/fastrtpsgen/java/
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp fastrtpsgen ${build_dir}/usr/bin/
	cp fastrtpsgen.jar ${build_dir}/usr/share/fastrtpsgen/java/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
        echo fastrtpsgen_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../fastrtpsgen_${version}_amd64.deb
	rm -rf ${build_dir}
	rm -f fastrtpsgen.jar fastrtpsgen

	echo "Done"
}

version=$(get_version)
build
make_deb
