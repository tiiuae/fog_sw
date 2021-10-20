#!/bin/bash

set -euxo pipefail

make_deb() {
	echo "Creating deb package fog-sw-systemd-services..."
	deb_dir=$(mktemp -d)

	mkdir -p ${deb_dir}/DEBIAN
	../../systemd/packaging/build.sh $(realpath ../../systemd) $(realpath ${deb_dir})

	upstream_version=1.0.0
	pushd ../../systemd
	git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
	popd
	version="${upstream_version}-${deb_revision}${git_version}"
	echo ${version}

	sed -i "s/VERSION/${version}/" ${deb_dir}/DEBIAN/control
	cat ${deb_dir}/DEBIAN/control
	fakeroot dpkg-deb --build ${deb_dir} ../fog-sw-systemd-services_${version}_amd64.deb
	rm -rf ${deb_dir}
	echo "Done"
}

deb_revision=${1:-0~dirty}
make_deb
