#!/bin/bash

source /opt/ros/galactic/setup.bash
echo "Start Mavlink Router"
mavlink-routerd >/fog-drone/mav_routerd_out.log 2>/fog-drone/mav_routerd_err.log &

echo "Start Micrortps_agent"
micrortps_agent -t UDP -n "$DRONE_DEVICE_ID" >/fog-drone/urtps_out.log 2>/fog-drone/urtps_err.log &

echo "Start Control Interface"
rm -Rf /fog-drone/control_interface_logs
mkdir -p /fog-drone/control_interface_logs
ROS_LOG_DIR=/fog-drone/control_interface_logs ros2 launch control_interface control_interface.py use_sim_time:=true >/fog-drone/control_interface_out.log 2>/fog-drone/control_interface_err.log &

echo "Start static tf"
rm -Rf /fog-drone/static_tf_logs
mkdir -p /fog-drone/static_tf_logs
ROS_LOG_DIR=/fog-drone/static_tf_logs ros2 launch rplidar_ros2 static_tf_launch.py >/fog-drone/static_tf_out.log 2>/fog-drone/static_tf_err.log &

echo "Start Navigation"
rm -Rf /fog-drone/navigation_logs
mkdir -p /fog-drone/navigation_logs
ROS_LOG_DIR=/fog-drone/navigation_logs ros2 launch navigation navigation.py use_sim_time:=true >/fog-drone/navigation_out.log 2>/fog-drone/navigation_err.log &

echo "Start Octomap"
rm -Rf /fog-drone/octomap_logs
mkdir -p /fog-drone/octomap_logs
ROS_LOG_DIR=/fog-drone/octomap_logs ros2 launch octomap_server2 octomap_server.py use_sim_time:=true >/fog-drone/octomap_out.log 2>/fog-drone/octomap_err.log &

echo "Start Video"
gst-launch-1.0 udpsrc port=5600 \
    ! "application/x-rtp" \
    ! rtph264depay \
    ! queue \
    ! video/x-h264 \
    ! rtspclientsink \
        name=sink \
        protocols=tcp \
        location="$RTSP_SERVER_ADDRESS/$DRONE_DEVICE_ID" \
        tls-validation-flags=0 \
    >/fog-drone/video_out.log \
    2>/fog-drone/video_err.log &

echo "Start mission data recorder"
mission-data-recorder \
    -device-id "$DRONE_DEVICE_ID" \
    -backend-url "$MISSION_DATA_RECORDER_BACKEND_URL" \
    -size-threshold "$MISSION_DATA_RECORDER_SIZE_THRESHOLD" \
    -topics "$MISSION_DATA_RECORDER_TOPICS" \
    -dest-dir /fog-drone/mission-data \
    >/fog-drone/mission-data-recorder_out.log \
    2>/fog-drone/mission-data-recorder_err.log &
echo "Start Mission Engine"
mission-engine -device_id "$DRONE_DEVICE_ID" >/fog-drone/mission-engine_out.log 2>/fog-drone/mission-engine_err.log &
echo "Start Communication link"
communication_link -device_id "$DRONE_DEVICE_ID" -mqtt_broker "$MQTT_BROKER_ADDRESS"
