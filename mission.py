#!/usr/bin/env python3

"""
Simple Mission Executor for Drone
Receives waypoint coordinates and executes mission via MAVSDK
"""

import asyncio
import sys
import json
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

class SimpleMissionExecutor:
    def __init__(self):
        self.drone = System()
        self.connected = False
        
    async def connect_drone(self, address="udp://:14540"):
        """Connect to drone"""
        try:
            print(f"Connecting to drone at {address}...")
            await self.drone.connect(system_address=address)
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    print("✓ Connected to drone")
                    return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    async def upload_and_start_mission(self, waypoints, speed=5.0):
        """Upload waypoints and start mission"""
        try:
            if not self.connected:
                print("✗ Not connected to drone")
                return False
            
            print(f"Creating mission with {len(waypoints)} waypoints...")
            
            # Clear existing mission
            await self.drone.mission.clear_mission()
            
            # Create mission items
            mission_items = []
            for i, wp in enumerate(waypoints):
                mission_item = MissionItem(
                    float(wp['lat']),                   # latitude_deg
                    float(wp['lon']),                   # longitude_deg
                    float(wp['alt']),                   # relative_altitude_m
                    speed,                              # speed_m_s
                    True,                               # is_fly_through
                    float('nan'),                       # gimbal_pitch_deg
                    float('nan'),                       # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,     # camera_action
                    float('nan'),                       # loiter_time_s
                    float('nan'),                       # camera_photo_interval_s
                    2.0,                                # acceptance_radius_m
                    float('nan'),                       # yaw_deg
                    float('nan'),                       # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE      # vehicle_action (required parameter)
                )
                mission_items.append(mission_item)
                print(f"  WP{i+1}: {wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']}m")
            
            # Upload mission
            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.upload_mission(mission_plan)
            print("✓ Mission uploaded")
            
            # Start mission
            await self.drone.mission.start_mission()
            print("✓ Mission started")
            
            # Monitor progress
            await self.monitor_mission()
            
            return True
            
        except Exception as e:
            print(f"✗ Mission failed: {e}")
            return False
    
    async def monitor_mission(self):
        """Monitor mission progress"""
        try:
            print("Monitoring mission progress...")
            async for progress in self.drone.mission.mission_progress():
                current = progress.current
                total = progress.total
                print(f"Progress: {current}/{total}")
                
                if current >= total and total > 0:
                    print("✓ Mission completed!")
                    break
        except Exception as e:
            print(f"✗ Mission monitoring error: {e}")

async def main():
    """Main function"""
    executor = SimpleMissionExecutor()
    
    # Connect to drone
    if not await executor.connect_drone():
        sys.exit(1)
    
    # Check if waypoints provided as argument
    if len(sys.argv) > 1:
        try:
            # Load waypoints from JSON string argument
            waypoints = json.loads(sys.argv[1])
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
        except (json.JSONDecodeError, ValueError) as e:
            print(f"✗ Invalid waypoints format: {e}")
            sys.exit(1)
    else:
        # Use test mission if no waypoints provided
        print("No waypoints provided, using test mission...")
        home_lat = 47.3977
        home_lon = 8.5456
        home_alt = 10.0
        offset = 0.000045  # ~5 meters
        
        waypoints = [
            {'lat': home_lat, 'lon': home_lon, 'alt': home_alt},
            {'lat': home_lat + offset, 'lon': home_lon, 'alt': home_alt},
            {'lat': home_lat + offset, 'lon': home_lon + offset, 'alt': home_alt},
            {'lat': home_lat, 'lon': home_lon + offset, 'alt': home_alt},
            {'lat': home_lat, 'lon': home_lon, 'alt': home_alt}
        ]
        speed = 3.0
    
    # Execute mission
    await executor.upload_and_start_mission(waypoints, speed)

if __name__ == "__main__":
    asyncio.run(main())