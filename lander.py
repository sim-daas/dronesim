#!/usr/bin/env python3

"""
Precision Landing Script with ArUco Marker Detection
Uses ROS2 for camera feed and MAVSDK for drone control with offboard mode
"""

import asyncio
import time
import threading
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# MAVSDK imports
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed

class PrecisionLander(Node):
    def __init__(self):
        super().__init__('precision_lander')
        print("🔧 Initializing precision lander components...")
        
        # Initialize components
        self.bridge = CvBridge()
        self.drone = System()
        print("✓ OpenCV bridge and MAVSDK system initialized")
        
        # ArUco detection setup (updated for newer OpenCV)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # 4x4 dictionary
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            print("✓ ArUco detector initialized with DICT_4X4_50")
        except Exception as e:
            print(f"⚠ ArUco initialization error: {e}")
            # Fallback to older API if available
            try:
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None  # Use legacy detection method
                print("✓ Using legacy ArUco API")
            except:
                raise Exception("Unable to initialize ArUco detector with any API version")
        
        # Marker parameters
        self.marker_size = 0.5  # 0.5m x 0.5m marker
        self.marker_detected = False
        self.marker_center_x = 0
        self.marker_center_y = 0
        self.image_width = 640
        self.image_height = 480
        
        # Control parameters
        self.max_velocity = 0.5  # 0.5 m/s max velocity
        self.landing_altitude = 5.0  # Start precision landing at 5m
        self.final_altitude = 1.0   # Final altitude before landing
        self.position_tolerance = 0.2  # 0.2m position tolerance
        self.detection_timeout = 20.0  # 20 seconds timeout
        
        # State variables
        self.current_position = {'lat': 0, 'lon': 0, 'alt': 0}
        self.landing_complete = False
        self.offboard_active = False
        
        # ROS2 subscriber
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',
            self.camera_callback,
            10
        )
        
        print("🎯 Precision Lander initialized")
        
    def camera_callback(self, msg):
        """Process camera images for ArUco marker detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # Detect ArUco markers (handle both new and legacy API)
            if self.aruco_detector is not None:
                # New API
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
            else:
                # Legacy API
                corners, ids, _ = cv2.aruco.detectMarkers(
                    cv_image, 
                    self.aruco_dict, 
                    parameters=self.aruco_params
                )
            
            if ids is not None and len(ids) > 0:
                # Use the first detected marker
                marker_corners = corners[0][0]
                
                # Calculate marker center
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                
                self.marker_center_x = center_x
                self.marker_center_y = center_y
                self.marker_detected = True
                
                # Optional: Draw marker on image for debugging
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f"Center: ({center_x}, {center_y})", 
                           (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            else:
                self.marker_detected = False
                
        except Exception as e:
            print(f"Camera callback error: {e}")
            self.marker_detected = False
    
    async def connect_to_drone(self):
        """Connect to drone via MAVSDK"""
        try:
            print("🔗 Connecting to drone...")
            await self.drone.connect(system_address="udp://:14540")
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("✓ Connected to drone")
                    break
                    
            # Get initial position
            async for position in self.drone.telemetry.position():
                self.current_position = {
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt': position.relative_altitude_m
                }
                print(f"📍 Initial position: {self.current_position['alt']:.1f}m altitude")
                break
                
            return True
            
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    async def descend_to_landing_altitude(self):
        """Descend to precision landing altitude (5-6m)"""
        try:
            print(f"⬇️  Descending to landing altitude ({self.landing_altitude}m)...")
            
            # Get current position for reference
            async for position in self.drone.telemetry.position():
                current_lat = position.latitude_deg
                current_lon = position.longitude_deg
                current_alt = position.relative_altitude_m
                print(f"📍 Current position: {current_lat:.6f}, {current_lon:.6f}, {current_alt:.1f}m")
                break
            
            # Skip descent if already at or below target altitude
            if current_alt <= self.landing_altitude + 1.0:
                print(f"✓ Already at target altitude: {current_alt:.1f}m (target: {self.landing_altitude}m)")
                return True
            
            print(f"📤 Sending goto command to: {current_lat:.6f}, {current_lon:.6f}, {self.landing_altitude}m")
            # Send goto command to same lat/lon but lower altitude
            await self.drone.action.goto_location(
                current_lat,
                current_lon,
                self.landing_altitude,
                0  # yaw
            )
            
            # Wait for descent with timeout
            descent_timeout = 30  # 30 seconds timeout
            start_time = time.time()
            
            while time.time() - start_time < descent_timeout:
                async for position in self.drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    print(f"📏 Current altitude: {current_alt:.1f}m (target: {self.landing_altitude}m)")
                    
                    if abs(current_alt - self.landing_altitude) < 1.0:
                        print(f"✓ Reached landing altitude: {current_alt:.1f}m")
                        return True
                    break
                await asyncio.sleep(2)  # Check every 2 seconds
            
            print(f"⏰ Descent timeout after {descent_timeout}s - proceeding anyway")
            return True  # Continue even if timeout
                
        except Exception as e:
            print(f"✗ Descent failed: {e}")
            return True  # Continue despite error
    
    async def start_offboard_mode(self):
        """Start offboard mode for precision control"""
        try:
            print("🎮 Starting offboard mode...")
            
            # Get current position for NED reference
            async for position in self.drone.telemetry.position():
                current_alt = position.relative_altitude_m
                print(f"📍 Setting NED reference at altitude: {current_alt:.1f}m")
                break
            
            print("📤 Setting initial position setpoint...")
            # Set initial setpoint (hover at current position)
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -self.landing_altitude, 0.0))
            
            print("🚀 Starting offboard mode...")
            # Start offboard mode
            await self.drone.offboard.start()
            self.offboard_active = True
            print("✓ Offboard mode active")
            
            # Small delay to let offboard mode stabilize
            await asyncio.sleep(1)
            return True
            
        except Exception as e:
            print(f"✗ Offboard mode failed: {e}")
            return False
    
    async def precision_landing_control(self):
        """Main precision landing control loop"""
        print("🎯 Starting precision landing control...")
        
        detection_start_time = time.time()
        last_log_time = time.time()
        landing_phase = "searching"  # phases: searching, adjusting, descending, landing
        target_altitude = self.landing_altitude
        loop_count = 0
        
        while not self.landing_complete:
            current_time = time.time()
            loop_count += 1
            
            # Timeout check
            elapsed_time = current_time - detection_start_time
            if elapsed_time > self.detection_timeout:
                print(f"⏰ Detection timeout after {elapsed_time:.1f}s - executing failsafe landing")
                await self.failsafe_landing()
                break
            
            # Log camera status every 5 seconds
            if current_time - last_log_time > 5.0:
                print(f"📷 Camera status: {'Marker detected' if self.marker_detected else 'No marker'} "
                      f"(timeout in {self.detection_timeout - elapsed_time:.1f}s)")
                last_log_time = current_time
            
            if self.marker_detected:
                # Reset timeout when marker is detected
                detection_start_time = current_time
                
                # Calculate position error in pixels
                image_center_x = self.image_width / 2
                image_center_y = self.image_height / 2
                
                pixel_error_x = self.marker_center_x - image_center_x
                pixel_error_y = self.marker_center_y - image_center_y
                
                # Convert pixel error to velocity commands
                velocity_scale_x = 0.002  # m/s per pixel
                velocity_scale_y = 0.002  # m/s per pixel
                
                velocity_x = -pixel_error_y * velocity_scale_y  # Forward/back (NED X)
                velocity_y = pixel_error_x * velocity_scale_x   # Left/right (NED Y)
                
                # Limit velocities
                velocity_x = max(-self.max_velocity, min(self.max_velocity, velocity_x))
                velocity_y = max(-self.max_velocity, min(self.max_velocity, velocity_y))
                
                # Check if we're centered enough
                pixel_distance = math.sqrt(pixel_error_x**2 + pixel_error_y**2)
                position_error_m = pixel_distance * 0.01  # Rough conversion to meters
                
                if position_error_m < self.position_tolerance:
                    if landing_phase == "searching":
                        landing_phase = "adjusting"
                        print("📍 Marker centered - starting descent")
                    elif landing_phase == "adjusting" and target_altitude > self.final_altitude:
                        landing_phase = "descending"
                        target_altitude = max(self.final_altitude, target_altitude - 0.5)
                        print(f"⬇️  Descending to {target_altitude:.1f}m")
                    elif target_altitude <= self.final_altitude:
                        landing_phase = "landing"
                        print("🛬 Final landing sequence")
                        await self.execute_final_landing()
                        break
                
                # Send velocity commands
                try:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(velocity_x, velocity_y, 0.1, 0.0)  # Slow descent
                    )
                except Exception as e:
                    print(f"⚠ Velocity command failed: {e}")
                
                # Log detailed info every 10 loops (about 1 second)
                if loop_count % 10 == 0:
                    print(f"🎯 Marker at ({self.marker_center_x}, {self.marker_center_y}), "
                          f"Error: {pixel_distance:.0f}px, "
                          f"Vel: ({velocity_x:.2f}, {velocity_y:.2f}), "
                          f"Phase: {landing_phase}")
                      
            else:
                # No marker detected - hover in place
                try:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                except Exception as e:
                    print(f"⚠ Hover command failed: {e}")
                
                # Log search status less frequently
                if loop_count % 50 == 0:  # Every 5 seconds
                    print(f"🔍 Searching for marker... ({elapsed_time:.1f}s elapsed)")
            
            await asyncio.sleep(0.1)  # 10Hz control loop
    
    async def execute_final_landing(self):
        """Execute final landing sequence"""
        try:
            print("🛬 Executing final landing...")
            
            # Stop offboard mode
            await self.drone.offboard.stop()
            self.offboard_active = False
            
            # Execute landing command
            await self.drone.action.land()
            print("✓ Landing command sent")
            
            # Monitor landing completion
            timeout = 30
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                async for armed in self.drone.telemetry.armed():
                    if not armed:
                        print("✓ Precision landing completed successfully!")
                        self.landing_complete = True
                        return
                    break
                await asyncio.sleep(1)
                
            print("⚠ Landing timeout - manual verification required")
            self.landing_complete = True
            
        except Exception as e:
            print(f"✗ Final landing error: {e}")
            self.landing_complete = True
    
    async def failsafe_landing(self):
        """Execute failsafe landing when marker is not detected"""
        try:
            print("🚨 Executing failsafe landing...")
            
            # Stop offboard mode if active
            if self.offboard_active:
                print("🛑 Stopping offboard mode...")
                await self.drone.offboard.stop()
                self.offboard_active = False
                print("✓ Offboard mode stopped")
                # Small delay to let mode transition
                await asyncio.sleep(1)
            
            print("🛬 Sending land command...")
            # Execute basic landing
            await self.drone.action.land()
            print("✓ Failsafe landing command sent - waiting for completion...")
            
            # Monitor landing completion
            timeout = 30
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    async for armed in self.drone.telemetry.armed():
                        if not armed:
                            print("✓ Failsafe landing completed successfully!")
                            self.landing_complete = True
                            return
                        break
                except Exception as e:
                    print(f"⚠ Telemetry check failed: {e}")
                    break
                    
                await asyncio.sleep(1)
                
            print("⏰ Landing timeout - marking as complete anyway")
            self.landing_complete = True
            
        except Exception as e:
            print(f"✗ Failsafe landing error: {e}")
            print("🚨 Emergency: Marking landing as complete to prevent hang")
            self.landing_complete = True

async def main():
    """Main function to run precision landing"""
    try:
        # Initialize ROS2
        rclpy.init()
        print("✓ ROS2 initialized successfully")
        
        # Create precision lander
        lander = PrecisionLander()
        print("✓ Precision lander created successfully")
        
        # Run ROS2 in separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(lander), daemon=True)
        ros_thread.start()
        
        print("🎯 Precision Landing System Starting...")
        
        # Connect to drone
        if not await lander.connect_to_drone():
            print("✗ Failed to connect to drone")
            return
        
        # Give ROS2 time to receive camera data
        print("📷 Waiting for camera feed...")
        await asyncio.sleep(2)  # Reduced from 3 to 2 seconds
        
        print(f"📊 Camera status: width={self.image_width}, height={self.image_height}")
        print(f"🎯 Detection timeout: {self.detection_timeout}s")
        
        # Descend to landing altitude
        if not await lander.descend_to_landing_altitude():
            print("✗ Failed to descend to landing altitude")
            await lander.failsafe_landing()
            return
        
        # Start offboard mode
        if not await lander.start_offboard_mode():
            print("✗ Failed to start offboard mode")
            await lander.failsafe_landing()
            return
        
        # Execute precision landing
        await lander.precision_landing_control()
        
        print("🎯 Precision landing sequence completed")
        
    except Exception as e:
        print(f"✗ Precision landing system error: {e}")
        try:
            if 'lander' in locals() and lander.offboard_active:
                await lander.drone.offboard.stop()
            if 'lander' in locals():
                await lander.failsafe_landing()
        except:
            pass
    
    finally:
        # Cleanup
        try:
            if 'lander' in locals():
                lander.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    asyncio.run(main())