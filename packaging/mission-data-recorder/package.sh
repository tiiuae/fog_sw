#!/bin/bash

set -euxo pipefail

upstream_version=1.0.0
deb_revision=${1:-0~dirty}
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
git_commit_hash=$(git rev-parse HEAD)
version="${upstream_version}-${deb_revision}${git_version}"
echo "version: ${version}"

THIS_DIR="$(dirname "$(readlink -f "$0")")"

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

cd "${THIS_DIR}"/../../mission-data-recorder

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/

cp ./packaging/debian/* "${build_dir}"/DEBIAN/
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
