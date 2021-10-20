#!/bin/bash

set -euxo pipefail

deb_revision=${1:-0~dirty}

THIS_DIR="$(dirname "$(readlink -f "$0")")"

pushd "${THIS_DIR}"/../../wpa

git_commit_hash=$(git rev-parse HEAD)
upstream_version=2:2.9.0
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)
version="${upstream_version}-${deb_revision}${git_version}"
echo ${version}

clog_tmp=$(mktemp)
mv debian/changelog "${clog_tmp}"

cat << EOF > ./debian/changelog
wpa (${version}) unstable; urgency=high
  * commit: ${git_commit_hash}
 -- $(grep -oP '(?<=Maintainer: ).*' ./debian/control)  $(date +'%a, %d %b %Y %H:%M:%S %z')
EOF

cat "${clog_tmp}" >> debian/changelog
rm "${clog_tmp}"

dpkg-buildpackage -rfakeroot -b
mv ../wpasupplicant_*.deb "${THIS_DIR}/../deb_files"
popd
