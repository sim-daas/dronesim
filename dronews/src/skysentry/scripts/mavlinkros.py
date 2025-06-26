#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import threading
from mavsdk import System
from std_msgs.msg import String

# Global control variables are removed
CONTROL_STEP = 0.5  # Change step value

class MavlinkManualControlNode(Node):
    def __init__(self, async_loop):
        super().__init__('mavlink_manual_control_node')
        self.async_loop = async_loop

        # Initialize MAVSDK system
        self.drone = System()

        # Instance control variables
        self.roll = 0.0
        self.pitch = 0.0
        self.throttle = 0.5
        self.yaw = 0.0

        # Create subscriber for keyboard input
        self.control_subscription = self.create_subscription(
            String,
            '/keyboard_input',
            self.control_callback,
            10)
        self.get_logger().info("Subscribed to /keyboard_input topic.")

        # Inform that the node is starting
        self.get_logger().info("Initializing MAVLink Manual Control Node...")

        # Schedule the drone connection and control tasks on the asyncio loop.
        asyncio.run_coroutine_threadsafe(self.run_drone(), self.async_loop)

    def control_callback(self, msg):
        """Handles incoming control commands from the /keyboard_input topic."""
        self.get_logger().info(f"Received: '{msg.data}' (type: {type(msg.data)})")
        keys = msg.data.split(',') if msg.data else []

        # Reset control values every loop
        self.roll, self.pitch, self.throttle, self.yaw = 0.0, 0.0, 0.5, 0.0
        value = CONTROL_STEP

        # Check keys and update control values
        if "LEFT" in keys:
            self.pitch = -value
        elif "RIGHT" in keys:
            self.pitch = value

        if "UP" in keys:
            self.roll = value
        elif "DOWN" in keys:
            self.roll = -value

        if "w" in keys:
            self.throttle = 0.8
        elif "s" in keys:
            self.throttle = 0.0

        if "a" in keys:
            self.yaw = -value
        elif "d" in keys:
            self.yaw = value

        if "r" in keys:
            self.get_logger().info("Arm command triggered via topic.")
            asyncio.run_coroutine_threadsafe(self.drone.action.arm(), self.async_loop)
        if "l" in keys:
            self.get_logger().info("Land command triggered via topic.")
            asyncio.run_coroutine_threadsafe(self.drone.action.land(), self.async_loop)
        if "i" in keys:
            asyncio.run_coroutine_threadsafe(self.print_flight_mode(), self.async_loop)

    async def run_drone(self):
        """Connect to the drone and start keyboard input and manual control tasks."""
        # Connect to the PX4 simulation over UDP.
        await self.drone.connect(system_address="udp://:14540")
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break

        # Wait until the health checks are good.
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Drone health OK for flying.")
                break

        # Start manual control loop. Input is handled by the subscriber.
        asyncio.create_task(self.manual_control_loop())

    async def print_flight_mode(self):
        async for flight_mode in self.drone.telemetry.flight_mode():
            self.get_logger().info(f"Flight Mode: {flight_mode}")
            break

    async def manual_control_loop(self):
        """
        Continuously send manual control input (roll, pitch, throttle, yaw) to the drone.
        """
        while rclpy.ok():
            self.get_logger().info(
                f"Manual Input: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Throttle={self.throttle:.2f}, Yaw={self.yaw:.2f}"
            )
            await self.drone.manual_control.set_manual_control_input(self.roll, self.pitch, self.throttle, self.yaw)
            await asyncio.sleep(0.1)

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
        node.get_logger().info("Shutting down MAVLink Manual Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Stop the asyncio loop and wait for the thread to finish.
        async_loop.call_soon_threadsafe(async_loop.stop)
        async_thread.join()

if __name__ == '__main__':
    main()
