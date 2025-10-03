#!/bin/bash

echo "=== PX4 Log Checker ==="
echo

# Check if PX4 is running
PX4_PID=$(pgrep -f "px4")
if [ -z "$PX4_PID" ]; then
    echo "❌ PX4 is not running"
    echo "Start PX4 SITL first with: make px4_sitl gazebo"
    exit 1
else
    echo "✅ PX4 is running (PID: $PX4_PID)"
fi

echo
echo "=== Recent PX4 Logs ==="
echo "Checking for mission-related logs..."

# Common locations for PX4 logs
PX4_LOG_DIRS=(
    "$HOME/.ros/log"
    "/tmp"
    "/var/log"
    "$HOME/PX4-Autopilot/build/px4_sitl_default/logs"
)

# Look for recent .ulg files (PX4 logs)
echo "Looking for recent .ulg log files..."
for dir in "${PX4_LOG_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        find "$dir" -name "*.ulg" -mtime -1 2>/dev/null | head -5
    fi
done

echo
echo "=== MAVLink Console Commands ==="
echo "You can connect to PX4 console with:"
echo "  mavlink_shell.py"
echo "  Or use QGroundControl MAVLink Console"
echo
echo "Useful commands to check mission status:"
echo "  mission status"
echo "  mission show"
echo "  commander status"
echo "  logger status"

echo
echo "=== Gazebo Check ==="
if pgrep -f "gzserver" > /dev/null; then
    echo "✅ Gazebo is running"
else
    echo "❌ Gazebo is not running"
fi

echo
echo "=== Network Connections ==="
echo "Checking UDP connections on port 14540..."
netstat -un | grep 14540 || echo "No connections found on port 14540"