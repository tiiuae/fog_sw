#!/bin/bash

export PX4_SIM_MODEL=ssrc_fog_x

working_dir="/fog-drone"

echo "starting PX4 instance"
px4 -d "/px4_sitl_etc" -w sitl_${PX4_SIM_MODEL} -s /px4_sitl_etc/init.d-posix/rcS > ${working_dir}/px4_out.log 2>${working_dir}/px4_err.log &
