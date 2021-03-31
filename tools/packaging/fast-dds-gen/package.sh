#!/bin/bash

get_version() {
    pushd ../../Fast-DDS-Gen/
    version=1.0.0~$(git describe --always --tags --dirty --match "v[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
    popd
}

build() {
	pushd ../../Fast-DDS-Gen/
	./gradlew build || exit 1
	cp build/libs/fastddsgen.jar ../packaging/fast-dds-gen/
	cp scripts/fastddsgen ../packaging/fast-dds-gen/
	popd
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/bin/
	mkdir -p ${build_dir}/usr/share/fastddsgen/java/
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	cp fastddsgen ${build_dir}/usr/bin/
	cp fastddsgen.jar ${build_dir}/usr/share/fastddsgen/java/

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
        echo fastddsgen_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../fastddsgen_${version}_amd64.deb
	rm -rf ${build_dir}
	rm -f fastddsgen.jar fastddsgen

	echo "Done"
}

version=$(get_version)
build
make_deb
