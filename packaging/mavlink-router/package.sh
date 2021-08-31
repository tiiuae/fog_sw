#!/bin/bash

get_version() {
    pushd ../../mavlink-router
    version=1.0.0$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
    echo ${version}
	popd
}

build() {
	pushd ../../mavlink-router
	./autogen.sh && ./configure CFLAGS='-g -O2' --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 --prefix=/usr --disable-systemd
	make || exit 1
	make DESTDIR=${build_dir} install
	popd
}

make_deb() {
	echo "Creating deb package..."

	mkdir ${build_dir}/DEBIAN
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/prerm ${build_dir}/DEBIAN/
	mkdir -p ${build_dir}/etc/mavlink-router/
	cp main.conf  ${build_dir}/etc/mavlink-router/
	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
#	cat ${build_dir}/DEBIAN/control
	echo mavlink_router_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../mavlink-router_${version}_amd64.deb
	echo "Done"
}

build_dir=$(mktemp -d)
version=$(get_version)
build
make_deb
rm -rf ${build_dir}
