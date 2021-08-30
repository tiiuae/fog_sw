#!/bin/bash

set -euxo pipefail

THIS_DIR="$(dirname "$(readlink -f "$0")")"

pushd "${THIS_DIR}"/../../ros2_ws/src/mesh_com/common/core/os/ubuntu/wpa_supplicant
dpkg-buildpackage -rfakeroot -b
mv ../wpasupplicant_*.deb "${THIS_DIR}/../deb_files"
popd
