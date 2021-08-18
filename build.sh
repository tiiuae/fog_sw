#!/bin/bash
set -euxo pipefail
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -f Dockerfile -t fogsw-builder:latest .
docker run -it --rm -v $PWD:/fog_sw fogsw-builder "/fog_sw/packaging/package_all.sh"
