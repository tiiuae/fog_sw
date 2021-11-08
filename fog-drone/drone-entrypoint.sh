#!/bin/bash

mkdir /enclave
echo "$DRONE_IDENTITY_KEY" > /enclave/rsa_private.pem

/fog-drone/run-px4.sh
/fog-drone/run-ros2.sh
