#!/usr/bin/env python3

"""
Isolated Mission Executor for Drone
Receives waypoint coordinates and executes mission via MAVSDK
Arms drone, uploads mission, starts execution with random interruption simulation
"""

import asyncio
import argparse
import sys
import json
import random
import time
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

class SimpleMissionExecutor:
    def __init__(self, sysid=1, grpc_port=50051, port=14540):
        self.drone = System(sysid=sysid, port=grpc_port)
        self.target_sysid = sysid
        self.connected = False
        self.port = port

    async def connect_drone(self, address=None):
        """Connect to drone and strictly verify SysID locking"""
        if address is None:
            address = f"udp://:{self.port}"
        try:
            print(f"Connecting to drone at {address} (Target SysID: {self.target_sysid})...")
            await self.drone.connect(system_address=address)
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    break
                    
            # STRICT VERIFICATION: Ensure MAVSDK Server didn't latch onto cross-talk
            try:
                actual_sysid = await self.drone.param.get_param_int('MAV_SYS_ID')
                if actual_sysid != self.target_sysid:
                    print(f"✗ CRITICAL ERROR: Connected to SysID {actual_sysid}, but expected {self.target_sysid}! PX4 Cross-talk detected.")
                    return False
                print(f"✓ Verified absolute connection to SysID {actual_sysid}")
            except Exception as e:
                print(f"⚠ Could not verify MAV_SYS_ID parameter: {e}")
                
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
            print("📍 Implementing automatic loop (100 repetitions) for continuous mission until battery low...")
            
            # Check drone status before uploading mission
            print("Checking drone position...")
            async for position in self.drone.telemetry.position():
                print(f"Position: {position.latitude_deg:.6f}, {position.longitude_deg:.6f}, {position.relative_altitude_m:.1f}m")
                break
            
            # Clear existing mission
            print("Clearing existing mission...")
            await self.drone.mission.clear_mission()
            print("✓ Existing mission cleared")
            
            # Create looped waypoints
            looped_waypoints = []
            if len(waypoints) > 0:
                looped_waypoints.append(waypoints[0])
            
            mission_waypoints = waypoints[1:] if len(waypoints) > 1 else waypoints
            for loop_num in range(100):
                for wp in mission_waypoints:
                    looped_waypoints.append(wp)
            
            print(f"✓ Mission expanded to {len(looped_waypoints)} waypoints")
            
            # Create mission items with explicit Takeoff command for the first waypoint
            mission_items = []
            for i, wp in enumerate(looped_waypoints):
                # Force the first action to be a TAKEOFF to prevent static ground holds
                action = MissionItem.VehicleAction.TAKEOFF if i == 0 else MissionItem.VehicleAction.NONE
                
                mission_item = MissionItem(
                    float(wp['lat']),                   
                    float(wp['lon']),                   
                    float(wp['alt']),                   
                    speed,                              
                    True,                               
                    float('nan'),                       
                    float('nan'),                       
                    MissionItem.CameraAction.NONE,      
                    float('nan'),                       
                    float('nan'),                       
                    2.0,                                
                    float('nan'),                       
                    float('nan'),                       
                    action                              
                )
                mission_items.append(mission_item)
            
            # UPLOAD FIRST. If you arm before this, the 10-second auto-disarm timeout will kill the flight.
            print("Uploading mission to drone (this takes several seconds)...")
            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.upload_mission(mission_plan)
            print("✓ Mission uploaded successfully")
            
            # Verify mission upload
            try:
                downloaded_mission = await self.drone.mission.download_mission()
                print(f"✓ Mission verified: {len(downloaded_mission.mission_items)} items on drone")
            except Exception as e:
                print(f"⚠ Mission verification failed: {e}")
            
            # ARM SECOND. Arm immediately before starting to beat the timeout clock.
            if not await self.arm_drone():
                print("✗ Failed to arm drone, aborting mission")
                return False
            
            # Start mission
            print("Starting mission...")
            await self.drone.mission.start_mission()
            print("✓ Mission started successfully")
            
            mission_duration = random.randint(90, 120)
            print(f"🎲 Mission will run for {mission_duration} seconds (simulating battery interruption)")
            
            # Monitor progress
            await self.monitor_mission_with_timeout(mission_duration)
            
            return True
            
        except Exception as e:
            print(f"✗ Mission failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def monitor_mission_with_timeout(self, timeout_seconds):
        """Monitor mission progress with timeout"""
        try:
            start_time = time.time()
            async for progress in self.drone.mission.mission_progress():
                current = progress.current
                total = progress.total
                elapsed_time = time.time() - start_time
                
                print(f"Progress: {current}/{total} | Time: {elapsed_time:.1f}s/{timeout_seconds}s")
                
                if current >= total and total > 0:
                    print("✓ Mission completed naturally!")
                    break
                    
                if elapsed_time >= timeout_seconds:
                    print("🔋 Battery low / Time limit reached - mission interrupted!")
                    print("BATTERY_LOW_SIGNAL") 
                    break
                    
        except Exception as e:
            print(f"✗ Mission monitoring error: {e}")

async def main():
    parser = argparse.ArgumentParser(description='Isolated Mission Executor')
    parser.add_argument('--waypoints', type=str, default=None, help='JSON string of waypoints')
    parser.add_argument('--speed', type=float, default=5.0, help='Mission speed in m/s')
    parser.add_argument('--port', type=int, default=14540, help='MAVLink UDP port')
    parser.add_argument('--sysid', type=int, default=1, help='Drone System ID')
    parser.add_argument('--grpc-port', type=int, default=50051, help='Unique gRPC port for this process')
    
    args = parser.parse_args()
    
    executor = SimpleMissionExecutor(port=args.port, sysid=args.sysid, grpc_port=args.grpc_port)
    
    if not await executor.connect_drone():
        sys.exit(1)
    
    if args.waypoints:
        try:
            waypoints = json.loads(args.waypoints)
            speed = args.speed
        except (json.JSONDecodeError, ValueError) as e:
            print(f"✗ Invalid waypoints format: {e}")
            sys.exit(1)
    else:
        print("No waypoints provided, using test mission...")
        home_lat, home_lon, home_alt = 47.3977, 8.5456, 10.0
        offset = 0.000045 
        waypoints = [
            {'lat': home_lat, 'lon': home_lon, 'alt': home_alt},
            {'lat': home_lat + offset, 'lon': home_lon, 'alt': home_alt},
            {'lat': home_lat + offset, 'lon': home_lon + offset, 'alt': home_alt},
            {'lat': home_lat, 'lon': home_lon + offset, 'alt': home_alt},
            {'lat': home_lat, 'lon': home_lon, 'alt': home_alt}
        ]
        speed = 3.0
    
    await executor.upload_and_start_mission(waypoints, speed)

if __name__ == "__main__":
    asyncio.run(main())