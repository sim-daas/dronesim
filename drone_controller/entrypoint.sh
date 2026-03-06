#!/bin/bash
set -e

echo "=== Drone Controller ${DRONE_ID} Starting ==="
echo "  SIM_HOST:           ${SIM_HOST:-simulation}"
echo "  PX4_PORT:           ${PX4_PORT:-14540}"
echo "  LOCAL_MAVLINK_PORT: ${LOCAL_MAVLINK_PORT:-14580}"
echo "  GRPC_PORT:          ${GRPC_PORT:-50051}"
echo "  API_PORT:           ${API_PORT:-5001}"
echo "  MAV_SYS_ID:         ${MAV_SYS_ID:-${DRONE_ID:-1}}"

# Generate mavlink-router config from template
echo "Generating mavlink-router config..."
sed \
    -e "s/SIM_HOST_PLACEHOLDER/${SIM_HOST:-simulation}/g" \
    -e "s/PX4_PORT_PLACEHOLDER/${PX4_PORT:-14540}/g" \
    -e "s/LOCAL_MAVLINK_PORT_PLACEHOLDER/${LOCAL_MAVLINK_PORT:-14580}/g" \
    /app/mavlink_router.conf.tmpl > /tmp/mavlink_router.conf

echo "Generated config:"
cat /tmp/mavlink_router.conf
echo ""

# Start mavlink-router in background
echo "Starting mavlink-router..."
mavlink-routerd -c /tmp/mavlink_router.conf &
MAVLINK_ROUTER_PID=$!
sleep 2

# Verify mavlink-router is running
if ! kill -0 $MAVLINK_ROUTER_PID 2>/dev/null; then
    echo "ERROR: mavlink-router failed to start"
    exit 1
fi
echo "✓ mavlink-router running (PID $MAVLINK_ROUTER_PID)"

# Start mavsdk_server in background
echo "Starting mavsdk_server on gRPC port ${GRPC_PORT}..."
mavsdk_server -p ${GRPC_PORT} udp://127.0.0.1:${LOCAL_MAVLINK_PORT:-14580} &
MAVSDK_PID=$!
sleep 3

# Verify mavsdk_server is running
if ! kill -0 $MAVSDK_PID 2>/dev/null; then
    echo "ERROR: mavsdk_server failed to start"
    exit 1
fi
echo "✓ mavsdk_server running (PID $MAVSDK_PID)"

# Start the controller (foreground — keeps container alive)
echo "Starting controller.py..."
exec python3 /app/controller.py
