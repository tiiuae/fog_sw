#!/bin/bash

set -euo pipefail

THISDIR="$(dirname "$(readlink -f "$0")")"

cd "${THISDIR}"

echo "Creating deb package linux-image..."
build_dir=$(mktemp -d)

pushd ${build_dir} > /dev/null
apt-get source linux-hwe-5.11-source-5.11.0
popd > /dev/null
cp -rf ../../fogsw_kernel_config/* ${build_dir}/
# The second parameter is ignored by the build script.
../../fogsw_kernel_config/packaging/build.sh $(realpath ${build_dir}) $(realpath ${build_dir}) "fog"

# Third parameter is the target directory where to copy the debian package.
../../fogsw_kernel_config/packaging/package.sh $(realpath ${build_dir}) "fog" "$(realpath $(dirname ${0}))/../"
rm -rf ${build_dir}
echo "Done"

exit 0
