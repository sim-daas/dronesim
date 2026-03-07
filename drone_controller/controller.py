#!/usr/bin/env python3

"""
Drone Controller — The "brain" for a single drone.

Connects to a specific PX4 instance via MAVSDK (through mavlink-router),
monitors telemetry, and exposes a REST API for the GCS to send commands.
"""

import asyncio
import os
import sys
import time
import math
import threading
import logging
from datetime import datetime

from flask import Flask, request, jsonify
from flask_cors import CORS
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

# ─── Configuration ─────────────────────────────────────────────────────────────

DRONE_ID = int(os.environ.get("DRONE_ID", 1))
GRPC_PORT = int(os.environ.get("GRPC_PORT", 50051))
LOCAL_MAVLINK_PORT = int(os.environ.get("LOCAL_MAVLINK_PORT", 14580))
API_PORT = int(os.environ.get("API_PORT", 5001))
MAV_SYS_ID = int(os.environ.get("MAV_SYS_ID", DRONE_ID))
BATTERY_RTL_THRESHOLD = float(os.environ.get("BATTERY_RTL_THRESHOLD", 20.0))

# ─── Logging ───────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format=f"[Drone-{DRONE_ID}] %(asctime)s %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(f"drone-{DRONE_ID}")

# ─── Telemetry Store ───────────────────────────────────────────────────────────

telemetry = {
    "drone_id": DRONE_ID,
    "connected": False,
    "armed": False,
    "flight_mode": "UNKNOWN",
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
    "ground_speed": 0.0,
    "battery_percent": 0.0,
    "battery_voltage": 0.0,
    "gps_fix_type": "No GPS",
    "gps_num_satellites": 0,
    "heading_deg": 0.0,
    "mission_progress": "0/0",
    "rtl_triggered": False,
    "last_update": None,
}
telemetry_lock = threading.Lock()

# Log buffer for GCS console
log_buffer = []
LOG_BUFFER_MAX = 200


def add_log(message):
    """Thread-safe log message storage."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    entry = f"[{timestamp}] {message}"
    log.info(message)
    with telemetry_lock:
        log_buffer.append(entry)
        if len(log_buffer) > LOG_BUFFER_MAX:
            log_buffer.pop(0)


# ─── MAVSDK Async Controller ──────────────────────────────────────────────────

drone = System(sysid=MAV_SYS_ID, port=GRPC_PORT)
event_loop = None
command_queue = asyncio.Queue()


async def connect_to_drone():
    """Connect to PX4 via MAVSDK server."""
    address = f"udp://:{LOCAL_MAVLINK_PORT}"
    add_log(f"Connecting to MAVSDK at {address} (SysID={MAV_SYS_ID}, gRPC={GRPC_PORT})...")

    try:
        await drone.connect(system_address=address)

        async for state in drone.core.connection_state():
            if state.is_connected:
                with telemetry_lock:
                    telemetry["connected"] = True
                add_log("✓ Connected to PX4")
                break
    except Exception as e:
        add_log(f"✗ Connection failed: {e}")
        return False

    # Verify SysID to detect crosstalk
    try:
        actual_sysid = await drone.param.get_param_int("MAV_SYS_ID")
        if actual_sysid != MAV_SYS_ID:
            add_log(f"✗ CRITICAL: Connected to SysID {actual_sysid}, expected {MAV_SYS_ID}! Crosstalk!")
            return False
        add_log(f"✓ Verified SysID={actual_sysid}")
    except Exception as e:
        add_log(f"⚠ Could not verify MAV_SYS_ID: {e}")

    return True


async def monitor_telemetry():
    """Run all telemetry monitors concurrently."""
    await asyncio.gather(
        _monitor_position(),
        _monitor_battery(),
        _monitor_flight_mode(),
        _monitor_armed(),
        _monitor_velocity(),
        _monitor_gps(),
        _monitor_heading(),
        _monitor_mission_progress(),
        _process_commands(),
    )


async def _monitor_position():
    counter = 0
    async for pos in drone.telemetry.position():
        with telemetry_lock:
            telemetry["lat"] = pos.latitude_deg
            telemetry["lon"] = pos.longitude_deg
            telemetry["alt"] = pos.relative_altitude_m
            telemetry["last_update"] = time.time()
        counter += 1
        if counter % 30 == 0:
            add_log(f"Position: {pos.latitude_deg:.6f}, {pos.longitude_deg:.6f}, {pos.relative_altitude_m:.1f}m")


async def _monitor_battery():
    async for battery in drone.telemetry.battery():
        percent = round(battery.remaining_percent * 100, 1)
        trigger_rtl = False
        
        with telemetry_lock:
            telemetry["battery_percent"] = percent
            telemetry["battery_voltage"] = round(battery.voltage_v, 2)
            
            # Fail-safe logic
            if percent < BATTERY_RTL_THRESHOLD and telemetry["armed"] and not telemetry["rtl_triggered"]:
                telemetry["rtl_triggered"] = True
                trigger_rtl = True
                
            # Reset flag if manually recharged/rearmed above threshold
            if percent >= BATTERY_RTL_THRESHOLD and telemetry["rtl_triggered"] and not telemetry["armed"]:
                telemetry["rtl_triggered"] = False
                
        if trigger_rtl:
            add_log(f"⚠ CRITICAL: Battery at {percent}%. Triggering automatic RTL!")
            # Execute RTL directly (bypassing queue for immediacy, though queue is also fine)
            try:
                await drone.action.return_to_launch()
            except Exception as e:
                add_log(f"✗ Auto-RTL failed: {e}")


async def _monitor_flight_mode():
    async for mode in drone.telemetry.flight_mode():
        with telemetry_lock:
            telemetry["flight_mode"] = mode.name


async def _monitor_armed():
    async for armed in drone.telemetry.armed():
        with telemetry_lock:
            telemetry["armed"] = armed


async def _monitor_velocity():
    async for vel in drone.telemetry.velocity_ned():
        speed = math.sqrt(vel.north_m_s ** 2 + vel.east_m_s ** 2)
        with telemetry_lock:
            telemetry["ground_speed"] = round(speed, 2)


async def _monitor_gps():
    fix_names = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
    async for gps in drone.telemetry.gps_info():
        with telemetry_lock:
            telemetry["gps_fix_type"] = fix_names.get(gps.fix_type, f"Unknown({gps.fix_type})")
            telemetry["gps_num_satellites"] = gps.num_satellites


async def _monitor_heading():
    async for heading in drone.telemetry.heading():
        with telemetry_lock:
            telemetry["heading_deg"] = round(heading.heading_deg, 1)


async def _monitor_mission_progress():
    async for progress in drone.mission.mission_progress():
        with telemetry_lock:
            telemetry["mission_progress"] = f"{progress.current}/{progress.total}"

# ─── Command Queue Processor ──────────────────────────────────────────────────

async def _process_commands():
    """Process commands sent from the Flask REST API."""
    while True:
        cmd = await command_queue.get()
        action = cmd.get("action")
        params = cmd.get("params", {})
        result_holder = cmd.get("result")

        try:
            if action == "arm":
                await drone.action.arm()
                add_log("✓ ARM command sent")
                result_holder["success"] = True
                result_holder["message"] = "Armed"

            elif action == "disarm":
                await drone.action.disarm()
                add_log("✓ DISARM command sent")
                result_holder["success"] = True
                result_holder["message"] = "Disarmed"

            elif action == "takeoff":
                alt = float(params.get("altitude", 5.0))
                await drone.action.set_takeoff_altitude(alt)
                await drone.action.takeoff()
                add_log(f"✓ TAKEOFF to {alt}m")
                result_holder["success"] = True
                result_holder["message"] = f"Taking off to {alt}m"

            elif action == "land":
                await drone.action.land()
                add_log("✓ LAND command sent")
                result_holder["success"] = True
                result_holder["message"] = "Landing"

            elif action == "rtl":
                await drone.action.return_to_launch()
                add_log("✓ RTL command sent")
                result_holder["success"] = True
                result_holder["message"] = "Returning to launch"

            elif action == "upload_mission":
                waypoints = params.get("waypoints", [])
                speed = float(params.get("speed", 5.0))
                
                if not waypoints:
                    raise ValueError("No waypoints provided")
                    
                await drone.mission.clear_mission()
                
                mission_items = []
                for wp in waypoints:
                    mission_item = MissionItem(
                        float(wp['lat']),                   
                        float(wp['lon']),                   
                        float(wp.get('alt', 10.0)),                   
                        speed,                              
                        True,                               
                        float('nan'),                       
                        float('nan'),                       
                        MissionItem.CameraAction.NONE,      
                        float('nan'),                       
                        float('nan'),                       
                        1.0,                                
                        float('nan'),                       
                        float('nan'),                       
                        MissionItem.VehicleAction.NONE                              
                    )
                    mission_items.append(mission_item)
                
                mission_plan = MissionPlan(mission_items)
                await drone.mission.upload_mission(mission_plan)
                
                add_log(f"✓ Mission uploaded ({len(mission_items)} waypoints)")
                result_holder["success"] = True
                result_holder["message"] = f"Uploaded {len(mission_items)} waypoints"

            elif action == "start_mission":
                try:
                    await drone.action.arm()
                    add_log("✓ Auto-armed drone for mission")
                except Exception as e:
                    add_log(f"⚠ Auto-arm skipped or failed: {e}")
                
                await drone.mission.start_mission()
                add_log("✓ Started mission execution")
                result_holder["success"] = True
                result_holder["message"] = "Mission started"

            elif action == "pause_mission":
                await drone.mission.pause_mission()
                add_log("✓ Paused mission")
                result_holder["success"] = True
                result_holder["message"] = "Mission paused"

            else:
                result_holder["success"] = False
                result_holder["message"] = f"Unknown action: {action}"

        except Exception as e:
            msg = f"Command '{action}' failed: {e}"
            add_log(f"✗ {msg}")
            result_holder["success"] = False
            result_holder["message"] = msg

        result_holder["done"] = True


def send_command(action, params=None):
    """Thread-safe command dispatch from Flask to the async MAVSDK loop."""
    result = {"success": False, "message": "", "done": False}
    cmd = {"action": action, "params": params or {}, "result": result}

    # Schedule onto the async event loop
    asyncio.run_coroutine_threadsafe(command_queue.put(cmd), event_loop)

    # Wait for the async handler to finish (max 10s)
    deadline = time.time() + 10
    while not result["done"] and time.time() < deadline:
        time.sleep(0.05)

    if not result["done"]:
        result["message"] = "Command timed out"
    return result


# ─── Flask REST API ────────────────────────────────────────────────────────────

app = Flask(__name__)
CORS(app)


@app.route("/status", methods=["GET"])
def get_status():
    """Quick health check."""
    with telemetry_lock:
        return jsonify({
            "drone_id": DRONE_ID,
            "connected": telemetry["connected"],
            "armed": telemetry["armed"],
            "flight_mode": telemetry["flight_mode"],
        })


@app.route("/telemetry", methods=["GET"])
def get_telemetry():
    """Full telemetry snapshot."""
    with telemetry_lock:
        return jsonify(dict(telemetry))


@app.route("/logs", methods=["GET"])
def get_logs():
    """Return recent log messages."""
    since = request.args.get("since", 0, type=int)
    with telemetry_lock:
        return jsonify({"logs": log_buffer[since:]})


@app.route("/arm", methods=["POST"])
def arm():
    result = send_command("arm")
    return jsonify(result), 200 if result["success"] else 500


@app.route("/disarm", methods=["POST"])
def disarm():
    result = send_command("disarm")
    return jsonify(result), 200 if result["success"] else 500


@app.route("/takeoff", methods=["POST"])
def takeoff():
    data = request.get_json(silent=True) or {}
    altitude = data.get("altitude", 5.0)
    result = send_command("takeoff", {"altitude": altitude})
    return jsonify(result), 200 if result["success"] else 500


@app.route("/land", methods=["POST"])
def land():
    result = send_command("land")
    return jsonify(result), 200 if result["success"] else 500


@app.route("/rtl", methods=["POST"])
def rtl():
    result = send_command("rtl")
    return jsonify(result), 200 if result["success"] else 500


@app.route("/mission/upload", methods=["POST"])
def upload_mission():
    data = request.get_json(silent=True) or {}
    waypoints = data.get("waypoints", [])
    speed = data.get("speed", 5.0)
    result = send_command("upload_mission", {"waypoints": waypoints, "speed": speed})
    return jsonify(result), 200 if result["success"] else 500


@app.route("/mission/start", methods=["POST"])
def start_mission():
    result = send_command("start_mission")
    return jsonify(result), 200 if result["success"] else 500


@app.route("/mission/pause", methods=["POST"])
def pause_mission():
    result = send_command("pause_mission")
    return jsonify(result), 200 if result["success"] else 500


# ─── Main ──────────────────────────────────────────────────────────────────────

def run_flask():
    """Run Flask in a separate thread."""
    app.run(host="0.0.0.0", port=API_PORT, debug=False, use_reloader=False)


async def main():
    global event_loop
    event_loop = asyncio.get_event_loop()

    add_log(f"Drone Controller {DRONE_ID} starting...")
    add_log(f"  MAVSDK gRPC port : {GRPC_PORT}")
    add_log(f"  Local MAVLink    : udp://:{LOCAL_MAVLINK_PORT}")
    add_log(f"  REST API port    : {API_PORT}")
    add_log(f"  MAV_SYS_ID       : {MAV_SYS_ID}")

    # Start Flask in background thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    add_log(f"REST API listening on 0.0.0.0:{API_PORT}")

    # Connect to MAVSDK
    connected = await connect_to_drone()
    if not connected:
        add_log("Failed to connect. REST API will keep running for GCS to see status.")
        # Keep alive so GCS can still poll /status
        while True:
            await asyncio.sleep(5)
            add_log("Waiting for PX4 connection...")
            connected = await connect_to_drone()
            if connected:
                break

    # Start telemetry monitoring loop
    await monitor_telemetry()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Shutting down")
