#!/bin/bash
set -e

echo "=== Simulation Container Starting ==="

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /root/dronews/install/setup.bash 2>/dev/null || true
source /root/.bashrc 2>/dev/null || true

# Copy tmuxinator config
mkdir -p ~/.config/tmuxinator
cp /app/sim.yml ~/.config/tmuxinator/sim.yml

# Check if QGroundControl exists and launch it
if [ -f "/app/QGC.AppImage" ]; then
    echo "Starting QGroundControl..."
    # Run as non-root user 'qgc' with X11 display access
    xhost +local:qgc >/dev/null 2>&1 || true
    sudo -u qgc --preserve-env=DISPLAY,XAUTHORITY /app/QGC.AppImage --appimage-extract-and-run > /tmp/qgc.log 2>&1 &
    sleep 3
fi

# Forward MAVLink offboard UDP ports to the Drone Controllers
echo "Setting up socat UDP bridges to Drone Controllers..."
socat UDP4-LISTEN:14540,fork UDP4:10.43.0.11:14540 &
socat UDP4-LISTEN:14541,fork UDP4:10.43.0.12:14541 &
socat UDP4-LISTEN:14542,fork UDP4:10.43.0.13:14542 &
socat UDP4-LISTEN:14543,fork UDP4:10.43.0.14:14543 &

# Start tmuxinator
echo "Starting simulation with tmuxinator..."
tmuxinator start sim

# Keep container alive
exec tail -f /dev/null
