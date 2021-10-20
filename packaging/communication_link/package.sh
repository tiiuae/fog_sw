#!/bin/bash

set -euxo pipefail

upstream_version=2.0.0
deb_revision=${1:-0~dirty}
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
git_commit_hash=$(git rev-parse HEAD)
version="${upstream_version}-${deb_revision}${git_version}"
echo "version: ${version}"

THIS_DIR="$(dirname "$(readlink -f "$0")")"

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

cd "${THIS_DIR}"/../../ros2_ws/src/communication_link

# build depedency to px4_msgs
export CGO_CFLAGS=
export CGO_LDFLAGS=
CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/include/)"
CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/lib/)"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/

pushd communicationlink
cp ./packaging/debian/* "${build_dir}"/DEBIAN/
go build -o communication_link
cp -f communication_link ${build_dir}/usr/bin/
popd


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
