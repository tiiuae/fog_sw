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

export CGO_CFLAGS=
export CGO_LDFLAGS=
# build depedency to px4_msgs
CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/include/)"
CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-foxy-px4-msgs/opt/ros/foxy/lib/)"
# build depedency to fog_msgs
CGO_CFLAGS="${CGO_CFLAGS} -I$(realpath ../fog_msgs/debian/ros-foxy-fog-msgs/opt/ros/foxy/include/)"
CGO_LDFLAGS="${CGO_LDFLAGS} -L$(realpath ../fog_msgs/debian/ros-foxy-fog-msgs/opt/ros/foxy/lib/)"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/
cp ./missionengine/packaging/debian/* "${build_dir}"/DEBIAN/

pushd ./missionengine/cmd
go build -o mission-engine
cp -f mission-engine "${build_dir}"/usr/bin/
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
