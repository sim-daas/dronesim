#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import threading
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from std_msgs.msg import String # Import String message type instead of Twist

# Global control variables are removed as state will be managed within the class

class MavlinkManualControlNode(Node):
    def __init__(self, async_loop):
        super().__init__('mavlink_manual_control_node')
        self.async_loop = async_loop

        # Initialize MAVSDK system
        self.drone = System()

        # Drone control state variables (NED coordinates and Yaw)
        self.north = 0.0
        self.east = 0.0
        self.down = 0.0 # Target altitude (negative for up in NED)
        self.yaw = 0.0 # Target yaw angle in degrees

        # Create subscriber for keyboard input strings
        self.control_subscription = self.create_subscription(
            String,             # Expect String messages
            '/keyboard_input',  # Topic name published by roskeypub.py
            self.control_callback,
            10)
        self.get_logger().info("Subscribed to /keyboard_input topic (String).")

        # Inform that the node is starting
        self.get_logger().info("Initializing MAVLink Offboard Control Node...")

        # Schedule the drone connection and control tasks on the asyncio loop.
        asyncio.run_coroutine_threadsafe(self.run_drone(), self.async_loop)

    def control_callback(self, msg):
        """Handles incoming String messages from the /keyboard_input topic."""
        # Parse the comma-separated string of pressed keys
        pressed_keys = msg.data.split(',') if msg.data else []

        # Reset changes for this callback
        north_change, east_change, down_change, yaw_change_deg = 0.0, 0.0, 0.0, 0.0
        value = 0.5  # Movement step size per interval
        yaw_rate = 2.0 # Yaw change in degrees per interval

        # Check for movement keys
        if "LEFT" in pressed_keys:
            east_change = -value
        elif "RIGHT" in pressed_keys:
            east_change = value

        if "UP" in pressed_keys:
            north_change = value
        elif "DOWN" in pressed_keys:
            north_change = -value

        if "w" in pressed_keys:
            down_change = -value  # Move up (decrease NED Down coordinate)
        elif "s" in pressed_keys:
            down_change = value   # Move down (increase NED Down coordinate)

        if "a" in pressed_keys:
            yaw_change_deg = -yaw_rate
        elif "d" in pressed_keys:
            yaw_change_deg = yaw_rate

        # Check for special command keys
        if "r" in pressed_keys:
             self.get_logger().info("Arm command received via topic.")
             # Run arming in the asyncio loop
             asyncio.run_coroutine_threadsafe(self.arm_drone(), self.async_loop)
        elif "l" in pressed_keys:
             self.get_logger().info("Land command received via topic.")
             # Run landing in the asyncio loop
             asyncio.run_coroutine_threadsafe(self.land_drone(), self.async_loop)
        elif "i" in pressed_keys: # Optional: print flight mode
             asyncio.run_coroutine_threadsafe(self.print_flight_mode(), self.async_loop)

        # Update position targets cumulatively only if movement keys were pressed
        # This prevents drift when no keys are pressed
        if north_change != 0 or east_change != 0 or down_change != 0 or yaw_change_deg != 0:
            self.north += north_change
            self.east += east_change
            self.down += down_change
            self.yaw += yaw_change_deg
            # Keep yaw within -180 to 180 degrees
            self.yaw = (self.yaw + 180) % 360 - 180

            self.get_logger().debug(f"Keys: {pressed_keys} -> NEDY Delta: N={north_change:.2f}, E={east_change:.2f}, D={down_change:.2f}, Y={yaw_change_deg:.1f}")
            self.get_logger().debug(f"New Target NEDY: N={self.north:.2f}, E={self.east:.2f}, D={self.down:.2f}, Y={self.yaw:.1f}")


    async def arm_drone(self):
        """Arms the drone."""
        try:
            await self.drone.action.arm()
            self.get_logger().info("Drone armed successfully!")
        except Exception as e:
            self.get_logger().error(f"Arming failed: {e}")

    async def land_drone(self):
        """Lands the drone and resets position targets."""
        try:
            await self.drone.action.land()
            self.get_logger().info("Landing command sent!")
            # Reset position targets on land command
            self.north, self.east, self.down, self.yaw = 0.0, 0.0, 0.0, 0.0
        except Exception as e:
            self.get_logger().error(f"Landing failed: {e}")


    async def run_drone(self):
        """Connect to the drone and start the offboard control task."""
        # Connect to the PX4 simulation over UDP.
        await self.drone.connect(system_address="udp://:14540")
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break

        # Wait until the health checks are good.
        self.get_logger().info("Waiting for drone to be ready...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
                self.get_logger().info("-- Drone health OK and ready to arm.")
                break
            await asyncio.sleep(1)


        # Start the offboard control loop.
        asyncio.create_task(self.offboard_control_loop()) # Renamed from manual_control_loop

    # Removed get_keyboard_input function

    async def print_flight_mode(self): # Keep this for debugging if needed
        async for flight_mode in self.drone.telemetry.flight_mode():
            self.get_logger().info(f"Flight Mode: {flight_mode}")
            break # Only print once when called

    async def offboard_control_loop(self): # Renamed from manual_control_loop
        """
        Handles the offboard control logic, sending position setpoints.
        """
        # Wait briefly
        await asyncio.sleep(2)

        self.get_logger().info("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.get_logger().error(f"Starting offboard mode failed with error code: {error._result.result}")
            self.get_logger().info("-- Disarming")
            await self.drone.action.disarm()
            # Attempt to arm first if starting offboard failed (common requirement)
            self.get_logger().info("-- Trying to arm and restart offboard")
            await self.arm_drone()
            await asyncio.sleep(1)
            try:
                 await self.drone.offboard.start()
                 self.get_logger().info("-- Offboard started after arming")
            except OffboardError as error:
                 self.get_logger().error(f"Starting offboard mode failed again: {error._result.result}")
                 return # Exit if it fails again

        # Start a task to periodically log the flight mode
        asyncio.ensure_future(self.log_flight_mode_periodically())

        # Main offboard loop
        while rclpy.ok():
            # Send the current target position from the instance variables
            try:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(self.north, self.east, self.down, self.yaw)
                )
                # Log sent position occasionally
                # self.get_logger().info(f"Sent NEDY: N={self.north:.2f}, E={self.east:.2f}, D={self.down:.2f}, Y={self.yaw:.1f}")
            except OffboardError as error:
                self.get_logger().error(f"Offboard set_position_ned failed: {error._result.result}")
                # Consider landing or holding position if control fails repeatedly
                await asyncio.sleep(1) # Avoid spamming errors
            except Exception as e:
                 self.get_logger().error(f"Failed to set offboard position: {e}")

            # Send commands at a reasonable rate (e.g., 5-10Hz)
            await asyncio.sleep(0.15) # Adjust rate as needed

    async def log_flight_mode_periodically(self):
        """Logs the flight mode every few seconds."""
        # ... (function remains the same as previous suggestion) ...
        while rclpy.ok():
            try:
                async for flight_mode in self.drone.telemetry.flight_mode():
                    self.get_logger().info(f"Current Flight Mode: {flight_mode}")
                    break # Get the current mode and break the inner loop
            except Exception as e:
                self.get_logger().warn(f"Could not get flight mode: {e}")
            await asyncio.sleep(5) # Log every 5 seconds


def main(args=None):
    rclpy.init(args=args)

    # Create a new asyncio event loop that runs in a separate thread.
    async_loop = asyncio.new_event_loop()
    async_thread = threading.Thread(target=lambda: async_loop.run_forever(), daemon=True)
    async_thread.start()

    node = MavlinkManualControlNode(async_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MAVLink Offboard Control Node...")
    finally:
        # Cleanly stop offboard mode before shutting down
        def stop_offboard_sync():
            try:
                # Ensure offboard is stopped using run_coroutine_threadsafe
                future = asyncio.run_coroutine_threadsafe(node.drone.offboard.stop(), async_loop)
                future.result(timeout=2) # Wait for result with timeout
                node.get_logger().info("-- Offboard stopped.")
            except Exception as e:
                node.get_logger().error(f"Failed to stop offboard mode: {e}")
            try:
                # Ensure drone is disarmed
                future = asyncio.run_coroutine_threadsafe(node.drone.action.disarm(), async_loop)
                future.result(timeout=2)
                node.get_logger().info("-- Drone disarmed.")
            except Exception as e:
                node.get_logger().error(f"Failed to disarm drone: {e}")

        if rclpy.ok(): # Check if context is still valid
             node.get_logger().info("Stopping offboard and disarming...")
             stop_offboard_sync()

        node.destroy_node()
        rclpy.shutdown()
        # Stop the asyncio loop and wait for the thread to finish.
        async_loop.call_soon_threadsafe(async_loop.stop)
        async_thread.join()

if __name__ == '__main__':
    main()

