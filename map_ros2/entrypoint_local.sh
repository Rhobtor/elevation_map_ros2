#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/local_cyclonedds.xml}"
export DDS_IP="${DDS_IP:-192.168.1.114}"
export DDS_EXTRA_INTERFACES="${DDS_EXTRA_INTERFACES:-}"
export DDS_PEERS="${DDS_PEERS:-}"

extra_interfaces=""
for iface in ${DDS_EXTRA_INTERFACES//,/ }; do
	[ -n "$iface" ] || continue
	extra_interfaces+="        <NetworkInterface address=\"${iface}\"/>\n"
done

peer_entries=""
for peer in ${DDS_PEERS//,/ }; do
	[ -n "$peer" ] || continue
	peer_entries+="        <Peer Address=\"${peer}\"/>\n"
done

if [ -n "$extra_interfaces" ]; then
	export CYCLONEDDS_EXTRA_INTERFACES
	CYCLONEDDS_EXTRA_INTERFACES=$(printf '%b' "$extra_interfaces")
else
	export CYCLONEDDS_EXTRA_INTERFACES=""
fi

if [ -n "$peer_entries" ]; then
	export CYCLONEDDS_PEERS
	CYCLONEDDS_PEERS=$(printf '<Peers>\n%b      </Peers>' "$peer_entries")
else
	export CYCLONEDDS_PEERS=""
fi

if [ -f /etc/cyclonedds/local_cyclonedds.template.xml ]; then
	envsubst < /etc/cyclonedds/local_cyclonedds.template.xml > /etc/cyclonedds/local_cyclonedds.xml
fi

echo "[entrypoint_local] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[entrypoint_local] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[entrypoint_local] DDS_IP=${DDS_IP}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"