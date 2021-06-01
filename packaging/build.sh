#!/bin/bash

build_dir=$1
dest_dir=$2

# Copy debian files from mod_specific directory
cp ${build_dir}/packaging/debian/* ${dest_dir}/DEBIAN/

exit 0