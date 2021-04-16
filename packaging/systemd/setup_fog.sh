#!/bin/sh

# Source local variables for this script
. /enclave/drone_device_id

# Export global environment variables
export DRONE_DEVICE_ID

# Enable SROS
DIR="/enclave/sros"
if [ -d "$DIR" ]; then
  echo "Folder ${DIR} exists. Enabling SROS2."
  export ROS_SECURITY_ENABLE=true
  export ROS_SECURITY_STRATEGY=Enforce
  export ROS_SECURITY_KEYSTORE="${DIR}"
  export ROS_SECURITY_LOG_VERBOSITY=WARN
  export ROS_SECURITY_LOG_FILE="${DIR}/log/ros-security.log"
else
  echo "Warning: ${DIR} not found. Will not enable SROS2."
fi

# Source ROS paths
. /opt/ros/foxy/setup.sh
