#!/bin/bash

pushd fast-rtps-gen
./package.sh
popd

pushd fast-rtps-lib
./package.sh
popd

pushd libsurvive
./package.sh
popd
