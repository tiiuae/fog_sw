#!/bin/bash

pushd fast-dds-gen
./package.sh
popd

pushd libsurvive
./package.sh
popd
