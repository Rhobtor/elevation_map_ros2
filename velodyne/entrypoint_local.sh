#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/local_cyclonedds.xml}"
export DDS_IP="${DDS_IP:-192.168.1.114}"

if [ -f /etc/cyclonedds/local_cyclonedds.template.xml ]; then
	envsubst < /etc/cyclonedds/local_cyclonedds.template.xml > /etc/cyclonedds/local_cyclonedds.xml
fi

echo "[entrypoint_local] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[entrypoint_local] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[entrypoint_local] DDS_IP=${DDS_IP}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"