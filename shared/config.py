"""
Shared configuration constants for the Surveillance Drone System.
Used by drone controllers and GCS to maintain consistent port/ID mappings.
"""

import os

# Number of drones in the fleet
NUM_DRONES = int(os.environ.get("NUM_DRONES", 4))

# Simulation container hostname (Docker DNS)
SIM_HOST = os.environ.get("SIM_HOST", "simulation")

# Per-drone configuration
# Each drone has: PX4 SITL UDP port, local MAVLink port, MAVSDK gRPC port, REST API port
DRONE_CONFIGS = {
    1: {"px4_port": 14540, "local_mavlink_port": 14580, "grpc_port": 50051, "api_port": 5001},
    2: {"px4_port": 14541, "local_mavlink_port": 14581, "grpc_port": 50052, "api_port": 5002},
    3: {"px4_port": 14542, "local_mavlink_port": 14582, "grpc_port": 50053, "api_port": 5003},
    4: {"px4_port": 14543, "local_mavlink_port": 14583, "grpc_port": 50054, "api_port": 5004},
}

# GCS configuration
GCS_PORT = int(os.environ.get("GCS_PORT", 8080))

# Drone controller hostnames (Docker DNS names)
def get_controller_host(drone_id):
    return os.environ.get(f"DRONE_{drone_id}_HOST", f"drone-controller-{drone_id}")

def get_controller_url(drone_id):
    config = DRONE_CONFIGS.get(drone_id, {})
    host = get_controller_host(drone_id)
    port = config.get("api_port", 5000 + drone_id)
    return f"http://{host}:{port}"

# UI Color scheme (shared between backends if needed)
COLORS = {
    "background": "#1C1C1C",
    "card": "#2C2C2C",
    "divider": "#444444",
    "primary_text": "#E0E0E0",
    "secondary_text": "#B0B0B0",
    "accent": "#1E90FF",
    "success": "#00FF85",
    "warning": "#FF6F61",
    "info": "#456882",
    "hover": "#3e3e42",
    "disabled": "#555555",
}
