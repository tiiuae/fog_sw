#!/bin/bash

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"

if ! go version > /dev/null 2>&1; then
  export PATH="/usr/local/go/bin:$PATH"
fi

cd "${THIS_DIR}"/../../ros2_ws/src/mission-data-recorder

upstream_version=$(git describe --tags HEAD --abbrev=0 --match='v[0-9]*' --always)
deb_revision=${1:-0~dirty}
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
git_commit_hash=$(git rev-parse HEAD)
[ "${upstream_version}" = "${git_commit_hash}" ] && upstream_version="v0.0.0"
upstream_version=$(echo ${upstream_version} | tail -c+2)
version="${upstream_version}-${deb_revision}${git_version}"
echo "version: ${version}"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/

cp ./packaging/debian/* "${build_dir}"/DEBIAN/
go generate ./...
go build -o mission-data-recorder
cp -f mission-data-recorder "${build_dir}"/usr/bin/

sed -i "s/VERSION/${version}/" "${build_dir}"/DEBIAN/control
cat "${build_dir}"/DEBIAN/control

### create changelog
pkg_name=$(grep -oP '(?<=Package: ).*' "${build_dir}"/DEBIAN/control)
mkdir -p "${build_dir}/usr/share/doc/${pkg_name}"
cat << EOF > "${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian"
${pkg_name} (${version}) focal; urgency=high

* commit: ${git_commit_hash}

-- $(grep -oP '(?<=Maintainer: ).*' ${build_dir}/DEBIAN/control)  $(date +'%a, %d %b %Y %H:%M:%S %z')

EOF
gzip "${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian"

### create debian package
debfilename=${pkg_name}_${version}_amd64.deb
echo "${debfilename}"
fakeroot dpkg-deb --build "${build_dir}" "${THIS_DIR}/../deb_files"

rm -rf "${build_dir}"
