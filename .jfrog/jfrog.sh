#!/bin/bash

set -euo pipefail

OUTDIR=$1
THISDIR="$(dirname "$(readlink -f "$0")")"

template="${THISDIR}"/spec.template
packages="${OUTDIR}"/packages
spec="${OUTDIR}"/spec.json

pushd "${THISDIR}/.."
declare -A modules
while read -r line
do
  sha=$(echo "$line" | awk '{print $1}' )
  submodule=$(echo "$line" | awk '{print $2}' )
  modules[$submodule]=$sha
done <<< "$(git submodule status)"
popd

mkdir -p "${packages}"
cp "${template}" "${spec}"

for submodule in "${!modules[@]}"
do
  sha="${modules[$submodule]}"
  sed -i "s|\${$submodule}|${sha}|" "${spec}"
done

pushd "${OUTDIR}"
jfrog rt dl --spec="${spec}"
popd
