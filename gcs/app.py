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
fleet_lock = threading.Lock()

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
        for drone_id, base_url in DRONE_CONTROLLERS.items():
            data = poll_drone_telemetry(drone_id, base_url)
            with fleet_lock:
                fleet_telemetry[drone_id] = data
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
    return _proxy_command(drone_id, "mission/upload", request.get_json(silent=True))


@app.route("/api/drone/<int:drone_id>/mission/start", methods=["POST"])
def start_mission(drone_id):
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
        return jsonify({"error": f"Drone {drone_id} not configured"}), 404

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
        return jsonify(result), resp.status_code
    except requests.exceptions.ConnectionError:
        msg = f"Drone {drone_id} controller unreachable"
        add_gcs_log(f"✗ {msg}")
        return jsonify({"success": False, "message": msg}), 503
    except requests.exceptions.Timeout:
        msg = f"Drone {drone_id} command '{action}' timed out"
        add_gcs_log(f"✗ {msg}")
        return jsonify({"success": False, "message": msg}), 504
    except Exception as e:
        msg = f"Error: {e}"
        add_gcs_log(f"✗ {msg}")
        return jsonify({"success": False, "message": msg}), 500


# ─── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    add_gcs_log(f"GCS starting with {NUM_DRONES} drones")
    for did, url in DRONE_CONTROLLERS.items():
        add_gcs_log(f"  Drone {did}: {url}")

    # Start background telemetry poller
    poller_thread = threading.Thread(target=telemetry_poller, daemon=True)
    poller_thread.start()

    app.run(host="0.0.0.0", port=GCS_PORT, debug=False)
