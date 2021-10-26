#!/bin/bash

set -eu

THISDIR="$(dirname "$(readlink -f "$0")")"

pushd "${THISDIR}/fogsw_kernel_config"
./package.sh
popd

pushd "${THISDIR}"
./package_sys.sh "$@"
./package_ros.sh "$@"
popd
