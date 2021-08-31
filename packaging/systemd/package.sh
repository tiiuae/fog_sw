#!/bin/bash

get_version() {
    pushd ../../systemd
    version=1.0.0$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
    echo ${version}
    popd
}

make_deb() {
	echo "Creating deb package fog-sw-systemd-services..."
	deb_dir=$(mktemp -d)

	mkdir -p ${deb_dir}/DEBIAN
	../../systemd/packaging/build.sh $(realpath ../../systemd) $(realpath ${deb_dir})

	get_version
	sed -i "s/VERSION/${version}/" ${deb_dir}/DEBIAN/control
	cat ${deb_dir}/DEBIAN/control
	fakeroot dpkg-deb --build ${deb_dir} ../fog-sw-systemd-services_${version}_amd64.deb
	rm -rf ${deb_dir}
	echo "Done"
}

version=$(get_version)
make_deb
