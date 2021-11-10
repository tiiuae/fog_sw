#!/bin/bash

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"

deb_revision=${1:-0~dirty}

echo "Creating deb package..."
build_dir=$(mktemp -d)

pushd $THIS_DIR/../../wifi-firmware
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
git_commit_hash=$(git rev-parse HEAD)
cp -rv DEBIAN "${build_dir}"
cp -rv lib "${build_dir}"
popd

upstream_version=1.0.0
version="${upstream_version}-${deb_revision}${git_version}"
echo ${version}

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

sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
cat ${build_dir}/DEBIAN/control
fakeroot dpkg-deb --build "${build_dir}" "${THIS_DIR}/../deb_files/${debfilename}"
rm -rf ${build_dir}
echo "Done"
