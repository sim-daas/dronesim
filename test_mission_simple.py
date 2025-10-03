#!/usr/bin/env python3

"""
Simple mission test to debug mission execution
"""

import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def main():
    drone = System()
    
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")
    
    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to drone")
            break
    
    print("\n=== TESTING MISSION UPLOAD ===")
    
    # Create a simple 2-waypoint mission
    home_lat = 47.3977
    home_lon = 8.5456
    home_alt = 10.0
    
    waypoints = [
        {'lat': home_lat, 'lon': home_lon, 'alt': home_alt},
        {'lat': home_lat + 0.0001, 'lon': home_lon + 0.0001, 'alt': home_alt}
    ]
    
    # Clear existing mission
    print("Clearing existing mission...")
    await drone.mission.clear_mission()
    
    # Create mission items
    mission_items = []
    for i, wp in enumerate(waypoints):
        mission_item = MissionItem(
            float(wp['lat']),
            float(wp['lon']),
            float(wp['alt']),
            5.0,                                # speed_m_s
            True,                               # is_fly_through
            float('nan'),                       # gimbal_pitch_deg
            float('nan'),                       # gimbal_yaw_deg
            MissionItem.CameraAction.NONE,
            float('nan'),                       # loiter_time_s
            float('nan'),                       # camera_photo_interval_s
            2.0,                                # acceptance_radius_m
            float('nan'),                       # yaw_deg
            float('nan'),                       # camera_photo_distance_m
            MissionItem.VehicleAction.NONE
        )
        mission_items.append(mission_item)
        print(f"Created waypoint {i+1}: {wp['lat']:.6f}, {wp['lon']:.6f}")
    
    # Upload mission
    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    print("✓ Mission uploaded")
    
    # Verify upload
    downloaded_mission = await drone.mission.download_mission()
    print(f"✓ Verified: {len(downloaded_mission.mission_items)} items on drone")
    
    print("\n=== CHECKING DRONE STATE FOR MISSION ===")
    
    # Check armed state
    async for armed in drone.telemetry.armed():
        print(f"Armed: {armed}")
        if not armed:
            print("⚠️ Drone is not armed - mission will not start until armed")
        break
        
    # Check flight mode
    async for flight_mode in drone.telemetry.flight_mode():
        print(f"Flight mode: {flight_mode}")
        break
    
    print("\n=== MISSION READY ===")
    print("Mission has been uploaded to the drone.")
    print("To start the mission:")
    print("1. Arm the drone (via dashboard or QGroundControl)")
    print("2. Use 'Start Mission' button in dashboard")
    print("3. Or start mission programmatically")

if __name__ == "__main__":
    asyncio.run(main())