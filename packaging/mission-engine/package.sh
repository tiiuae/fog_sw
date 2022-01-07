#!/bin/bash

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"

if ! go version > /dev/null 2>&1; then
  export PATH="/usr/local/go/bin:$PATH"
fi

cd "${THIS_DIR}"/../../ros2_ws/src/mission-engine

upstream_version=$(git describe --tags HEAD --abbrev=0 --match='v*' | tail -c+2)
deb_revision=${1:-0~dirty}
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
git_commit_hash=$(git rev-parse HEAD)
version="${deb_revision}${git_version}"
echo "version: ${version}"

export CGO_CFLAGS=
export CGO_LDFLAGS=
# build depedency to px4_msgs
CGO_CFLAGS="-I$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/include/)"
CGO_LDFLAGS="-L$(realpath ../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic/lib/)"
# build depedency to fog_msgs
CGO_CFLAGS="${CGO_CFLAGS} -I$(realpath ../fog_msgs/debian/ros-galactic-fog-msgs/opt/ros/galactic/include/)"
CGO_LDFLAGS="${CGO_LDFLAGS} -L$(realpath ../fog_msgs/debian/ros-galactic-fog-msgs/opt/ros/galactic/lib/)"

build_dir=$(mktemp -d)
mkdir -p "${build_dir}"/DEBIAN
mkdir -p "${build_dir}"/usr/bin/
cp ./packaging/debian/* "${build_dir}"/DEBIAN/

PX4_PATH="../px4_msgs/debian/ros-galactic-px4-msgs/opt/ros/galactic"
FOG_PATH="../fog_msgs/debian/ros-galactic-fog-msgs/opt/ros/galactic"
go generate ./...
go run github.com/tiiuae/rclgo/cmd/rclgo-gen generate -d internal/msgs -r ${PX4_PATH} ./...
go run github.com/tiiuae/rclgo/cmd/rclgo-gen generate -d internal/msgs -r ${FOG_PATH} ./...
go build -o mission-engine ./cmd
cp -f mission-engine "${build_dir}"/usr/bin/

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
