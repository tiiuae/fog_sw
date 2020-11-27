#!/bin/sh

# Source local variables for this script
. /enclave/drone_device_id

# Export global environment variables
export DRONE_DEVICE_ID

# Source ROS paths
. /opt/ros/foxy/setup.sh
