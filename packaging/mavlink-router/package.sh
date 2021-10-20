#!/bin/bash

set -euxo pipefail

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

	upstream_version=1.0.0
	pushd ../../mavlink-router
	git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
	popd
	version="${upstream_version}-${deb_revision}${git_version}"
	echo ${version}

	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
#	cat ${build_dir}/DEBIAN/control
	echo mavlink_router_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../mavlink-router_${version}_amd64.deb
	echo "Done"
}

deb_revision=${1:-0~dirty}
build_dir=$(mktemp -d)
build
make_deb
rm -rf ${build_dir}
