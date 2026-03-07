#!/usr/bin/env python3


"""
GCS Web Dashboard — Flask backend.

Aggregates telemetry from all drone controllers via their REST APIs
and serves a web-based dashboard for fleet monitoring and control.
"""

import os
import time
import threading
import logging
import random
import traceback
from datetime import datetime

import requests
from flask import Flask, jsonify, request, render_template, send_from_directory

# ─── Configuration ─────────────────────────────────────────────────────────────

NUM_DRONES = int(os.environ.get("NUM_DRONES", 4))
GCS_PORT = int(os.environ.get("GCS_PORT", 8080))

# Build drone controller URLs from environment or defaults
DRONE_CONTROLLERS = {}
for i in range(1, NUM_DRONES + 1):
    host = os.environ.get(f"DRONE_{i}_HOST", f"drone-controller-{i}")
    port = int(os.environ.get(f"DRONE_{i}_PORT", 5000 + i))
    DRONE_CONTROLLERS[i] = f"http://{host}:{port}"

# ─── Logging ───────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="[GCS] %(asctime)s %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gcs")

# ─── Telemetry Cache ───────────────────────────────────────────────────────────

fleet_telemetry = {}
fleet_lock = threading.RLock()

# State for relay handover
fleet_state = {
    i: {
        "nummissions": 0,
        "mission_waypoints": [],
        "mission_speed": 5.0,
        "mission_start_time": None,
        "battery_limit": None,
        "handoff_triggered": False
    } for i in range(1, NUM_DRONES + 1)
}

# GCS-level log buffer
gcs_logs = []
GCS_LOG_MAX = 300


def add_gcs_log(message):
    timestamp = datetime.now().strftime("%H:%M:%S")
    entry = f"[{timestamp}] {message}"
    log.info(message)
    with fleet_lock:
        gcs_logs.append(entry)
        if len(gcs_logs) > GCS_LOG_MAX:
            gcs_logs.pop(0)


def poll_drone_telemetry(drone_id, base_url):
    """Poll a single drone controller for telemetry."""
    try:
        resp = requests.get(f"{base_url}/telemetry", timeout=2)
        if resp.status_code == 200:
            data = resp.json()
            data["reachable"] = True
            return data
    except requests.exceptions.ConnectionError:
        pass
    except requests.exceptions.Timeout:
        pass
    except Exception as e:
        add_gcs_log(f"Error polling drone {drone_id}: {e}")

    return {
        "drone_id": drone_id,
        "connected": False,
        "reachable": False,
        "armed": False,
        "flight_mode": "UNKNOWN",
        "lat": 0.0, "lon": 0.0, "alt": 0.0,
        "ground_speed": 0.0,
        "battery_percent": 0.0,
        "battery_voltage": 0.0,
        "gps_fix_type": "No GPS",
        "gps_num_satellites": 0,
        "heading_deg": 0.0,
        "last_update": None,
    }


def telemetry_poller():
    """Background thread: poll all drone controllers every 1s."""
    while True:
        new_data = {}
        for drone_id, base_url in DRONE_CONTROLLERS.items():
            new_data[drone_id] = poll_drone_telemetry(drone_id, base_url)
            
        with fleet_lock:
            for drone_id, data in new_data.items():
                if drone_id not in fleet_state:
                    # In case dynamic drones are added later (not expected right now)
                    fleet_state[drone_id] = {"nummissions": 0, "mission_waypoints": [], "mission_speed": 5.0, "mission_start_time": None, "battery_limit": None, "handoff_triggered": False}
                
                state = fleet_state[drone_id]
                data["nummissions"] = state["nummissions"]
                fleet_telemetry[drone_id] = data

                # Relay Handover Logic
                if data["armed"] and state["mission_start_time"] is not None and state["battery_limit"] is not None:
                    elapsed = time.time() - state["mission_start_time"]
                    
                    # 1. Trigger handoff at T-10s
                    if elapsed >= (state["battery_limit"] - 10) and not state["handoff_triggered"]:
                        state["handoff_triggered"] = True
                        add_gcs_log(f"⚠ Drone {drone_id} running low on simulated battery! Initiating handoff.")
                        
                        # Find relief drone
                        relief_drone = None
                        min_missions = float('inf')
                        for cand_id, cand_data in fleet_telemetry.items():
                            if cand_id == drone_id: continue
                            if cand_data.get("connected") and not cand_data.get("armed"):
                                if fleet_state[cand_id]["nummissions"] < min_missions:
                                    min_missions = fleet_state[cand_id]["nummissions"]
                                    relief_drone = cand_id
                        
                        if relief_drone:
                            add_gcs_log(f"✓ Selected Drone {relief_drone} as relief (Missions: {min_missions})")
                            
                            prog_str = data.get("mission_progress", "0/0")
                            try:
                                current_wp = int(prog_str.split('/')[0])
                            except:
                                current_wp = 0
                            
                            # Safely prevent out of bounds
                            if current_wp >= len(state["mission_waypoints"]):
                                current_wp = max(0, len(state["mission_waypoints"]) - 1)
                                
                            rem_wps = state["mission_waypoints"][current_wp:]
                            
                            if rem_wps:
                                threading.Thread(target=_execute_handoff, args=(relief_drone, rem_wps, state["mission_speed"])).start()
                            else:
                                add_gcs_log(f"⚠ Drone {drone_id} has no remaining waypoints to hand off.")
                        else:
                            add_gcs_log(f"✗ RELAY FAILED: No available relief drones for Drone {drone_id}!")

                    # 2. Trigger RTL at T-0s (battery dead)
                    if elapsed >= state["battery_limit"]:
                        add_gcs_log(f"🔋 Drone {drone_id} simulated battery depleted. Forcing RTL.")
                        state["mission_start_time"] = None # prevent re-triggering
                        threading.Thread(target=_proxy_command, args=(drone_id, "rtl")).start()
                        
        time.sleep(1)


# ─── Flask App ─────────────────────────────────────────────────────────────────

app = Flask(__name__, static_folder="static", template_folder="templates")


@app.route("/")
def dashboard():
    return render_template("index.html", num_drones=NUM_DRONES)


@app.route("/api/fleet", methods=["GET"])
def get_fleet():
    """Return telemetry for all drones."""
    with fleet_lock:
        return jsonify({"drones": fleet_telemetry, "timestamp": time.time()})


@app.route("/api/drone/<int:drone_id>/telemetry", methods=["GET"])
def get_drone_telemetry(drone_id):
    """Return telemetry for a specific drone."""
    with fleet_lock:
        data = fleet_telemetry.get(drone_id)
    if data:
        return jsonify(data)
    return jsonify({"error": f"Drone {drone_id} not found"}), 404


@app.route("/api/drone/<int:drone_id>/arm", methods=["POST"])
def arm_drone(drone_id):
    return _proxy_command(drone_id, "arm")


@app.route("/api/drone/<int:drone_id>/disarm", methods=["POST"])
def disarm_drone(drone_id):
    return _proxy_command(drone_id, "disarm")


@app.route("/api/drone/<int:drone_id>/takeoff", methods=["POST"])
def takeoff_drone(drone_id):
    return _proxy_command(drone_id, "takeoff", request.get_json(silent=True))


@app.route("/api/drone/<int:drone_id>/land", methods=["POST"])
def land_drone(drone_id):
    return _proxy_command(drone_id, "land")


@app.route("/api/drone/<int:drone_id>/rtl", methods=["POST"])
def rtl_drone(drone_id):
    return _proxy_command(drone_id, "rtl")


@app.route("/api/drone/<int:drone_id>/mission/upload", methods=["POST"])
def upload_mission(drone_id):
    payload = request.get_json(silent=True)
    if payload:
        with fleet_lock:
            fleet_state[drone_id]["mission_waypoints"] = payload.get("waypoints", [])
            fleet_state[drone_id]["mission_speed"] = payload.get("speed", 5.0)
    return _proxy_command(drone_id, "mission/upload", payload)


@app.route("/api/drone/<int:drone_id>/mission/start", methods=["POST"])
def start_mission(drone_id):
    with fleet_lock:
        fleet_state[drone_id]["mission_start_time"] = time.time()
        fleet_state[drone_id]["battery_limit"] = random.randint(50, 60)
        fleet_state[drone_id]["handoff_triggered"] = False
        fleet_state[drone_id]["nummissions"] += 1
    return _proxy_command(drone_id, "mission/start")


@app.route("/api/drone/<int:drone_id>/mission/pause", methods=["POST"])
def pause_mission(drone_id):
    return _proxy_command(drone_id, "mission/pause")


@app.route("/api/logs", methods=["GET"])
def get_gcs_logs():
    """Return GCS-level logs."""
    since = request.args.get("since", 0, type=int)
    with fleet_lock:
        return jsonify({"logs": gcs_logs[since:]})


@app.route("/api/drone/<int:drone_id>/logs", methods=["GET"])
def get_drone_logs(drone_id):
    """Proxy log request to a specific drone controller."""
    base_url = DRONE_CONTROLLERS.get(drone_id)
    if not base_url:
        return jsonify({"error": f"Drone {drone_id} not found"}), 404
    try:
        since = request.args.get("since", 0, type=int)
        resp = requests.get(f"{base_url}/logs", params={"since": since}, timeout=3)
        return jsonify(resp.json())
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def _proxy_command(drone_id, action, payload=None):
    """Forward a command to a drone controller."""
    base_url = DRONE_CONTROLLERS.get(drone_id)
    if not base_url:
        return {"error": f"Drone {drone_id} not configured"}, 404

    add_gcs_log(f"Sending '{action}' to Drone {drone_id}")

    try:
        resp = requests.post(
            f"{base_url}/{action}",
            json=payload,
            timeout=15,
        )
        result = resp.json()
        status = "✓" if result.get("success") else "✗"
        add_gcs_log(f"{status} Drone {drone_id} {action}: {result.get('message', '')}")
        return result, resp.status_code
    except requests.exceptions.ConnectionError:
        msg = f"Drone {drone_id} controller unreachable"
        add_gcs_log(f"✗ {msg}")
        return {"success": False, "message": msg}, 503
    except requests.exceptions.Timeout:
        msg = f"Drone {drone_id} command '{action}' timed out"
        add_gcs_log(f"✗ {msg}")
        return {"success": False, "message": msg}, 504
    except Exception as e:
        msg = f"Error: {e}"
        add_gcs_log(f"✗ {msg}")
        str_trace = traceback.format_exc()
        for line in str_trace.split('\n'):
            log.error(line)
        return {"success": False, "message": msg}, 500


def _execute_handoff(relief_drone, waypoints, speed):
    """Background helper to dispatch the relief drone."""
    payload = {"waypoints": waypoints, "speed": speed}
    
    with fleet_lock:
        fleet_state[relief_drone]["mission_waypoints"] = waypoints
        fleet_state[relief_drone]["mission_speed"] = speed
        fleet_state[relief_drone]["mission_start_time"] = time.time()
        fleet_state[relief_drone]["battery_limit"] = random.randint(50, 60)
        fleet_state[relief_drone]["handoff_triggered"] = False
        fleet_state[relief_drone]["nummissions"] += 1
        
    add_gcs_log(f"Uploading spliced mission ({len(waypoints)} wps) to Drone {relief_drone}")
    _proxy_command(relief_drone, "mission/upload", payload)
    
    # Optional delay to allow mission digestion
    time.sleep(1)
    
    _proxy_command(relief_drone, "mission/start")


# ─── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    add_gcs_log(f"GCS starting with {NUM_DRONES} drones")
    for did, url in DRONE_CONTROLLERS.items():
        add_gcs_log(f"  Drone {did}: {url}")

    # Start background telemetry poller
    poller_thread = threading.Thread(target=telemetry_poller, daemon=True)
    poller_thread.start()

    app.run(host="0.0.0.0", port=GCS_PORT, debug=False)
