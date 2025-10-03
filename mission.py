#!/usr/bin/env python3

"""
Simple Mission Executor for Drone
Receives waypoint coordinates and executes mission via MAVSDK
Arms drone, uploads mission, starts execution with random interruption simulation
"""

import asyncio
import sys
import json
import random
import time
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
    
    async def arm_drone(self, max_retries=3):
        """Arm the drone with retries"""
        for attempt in range(max_retries):
            try:
                print(f"Arming drone (attempt {attempt + 1}/{max_retries})...")
                
                # Check if already armed
                async for armed in self.drone.telemetry.armed():
                    if armed:
                        print("✓ Drone is already armed")
                        return True
                    break
                
                # Arm the drone
                await self.drone.action.arm()
                
                # Wait for confirmation
                await asyncio.sleep(2)
                
                # Verify arming
                async for armed in self.drone.telemetry.armed():
                    if armed:
                        print("✓ Drone armed successfully")
                        return True
                    break
                    
                print(f"⚠ Arming attempt {attempt + 1} failed")
                if attempt < max_retries - 1:
                    await asyncio.sleep(2)
                    
            except Exception as e:
                print(f"✗ Arming attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    await asyncio.sleep(2)
        
        print("✗ Failed to arm drone after all attempts")
        return False
    
    async def upload_and_start_mission(self, waypoints, speed=5.0):
        """Upload waypoints and start mission"""
        try:
            if not self.connected:
                print("✗ Not connected to drone")
                return False
            
            print(f"Creating mission with {len(waypoints)} waypoints...")
            
            # Check drone status before uploading mission
            print("Checking drone status...")
                
            # Check position
            async for position in self.drone.telemetry.position():
                print(f"Position: {position.latitude_deg:.6f}, {position.longitude_deg:.6f}, {position.relative_altitude_m:.1f}m")
                break
            
            # Arm the drone first
            if not await self.arm_drone():
                print("✗ Failed to arm drone, aborting mission")
                return False
            
            # Clear existing mission
            print("Clearing existing mission...")
            await self.drone.mission.clear_mission()
            print("✓ Existing mission cleared")
            
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
            print("Uploading mission to drone...")
            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.upload_mission(mission_plan)
            print("✓ Mission uploaded successfully")
            
            # Verify mission upload
            print("Verifying mission upload...")
            try:
                downloaded_mission = await self.drone.mission.download_mission()
                print(f"✓ Mission verified: {len(downloaded_mission.mission_items)} items on drone")
            except Exception as e:
                print(f"⚠ Mission verification failed: {e}")
            
            # Check if we need to arm the drone
            async for armed in self.drone.telemetry.armed():
                if not armed:
                    print("⚠ Drone is not armed - mission may not start properly")
                    print("Please arm the drone manually or via the dashboard")
                else:
                    print("✓ Drone is armed and ready")
                break
            
            # Start mission
            print("Starting mission...")
            await self.drone.mission.start_mission()
            print("✓ Mission started successfully")
            
            # Generate random mission duration (20-40 seconds)
            mission_duration = random.randint(20, 40)
            print(f"🎲 Mission will run for {mission_duration} seconds (simulating battery/interruption)")
            
            # Monitor progress with timeout
            await self.monitor_mission_with_timeout(mission_duration)
            
            return True
            
        except Exception as e:
            print(f"✗ Mission failed: {e}")
            import traceback
            print("Full error traceback:")
            traceback.print_exc()
            return False
    
    async def monitor_mission_with_timeout(self, timeout_seconds):
        """Monitor mission progress with timeout"""
        try:
            print("Monitoring mission progress...")
            start_time = time.time()
            
            async for progress in self.drone.mission.mission_progress():
                current = progress.current
                total = progress.total
                elapsed_time = time.time() - start_time
                
                print(f"Progress: {current}/{total} | Time: {elapsed_time:.1f}s/{timeout_seconds}s")
                
                # Check if mission completed naturally
                if current >= total and total > 0:
                    print("✓ Mission completed naturally!")
                    print("🏠 Mission finished - dashboard will handle return to home and landing")
                    break
                    
                # Check if timeout reached (battery simulation)
                if elapsed_time >= timeout_seconds:
                    print("🔋 Battery low / Time limit reached - mission interrupted!")
                    print("🏠 Dashboard will take over for return to home and landing")
                    break
                    
        except Exception as e:
            print(f"✗ Mission monitoring error: {e}")
    
    async def monitor_mission(self):
        """Monitor mission progress (legacy method)"""
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