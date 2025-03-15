'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import threading
from mavsdk import System

class MavlinkManualControlNode(Node):
    def __init__(self, async_loop):
        super().__init__('mavlink_manual_control_node')
        self.async_loop = async_loop

        # Initialize the MAVSDK drone system
        self.drone = System()

        # Subscribe to the keyboard input topic
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.get_logger().info("MAVLink Manual Control Node Initialized.")

        # Start connection to the drone asynchronously
        asyncio.run_coroutine_threadsafe(self.connect_drone(), self.async_loop)

    async def connect_drone(self):
        """Connect to the drone via UDP."""
        await self.drone.connect(system_address="udp://:14540")
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Drone health OK for flying.")
                break

    async def arm_drone(self):
        """Arm the drone."""
        self.get_logger().info("Arming drone...")
        await self.drone.action.arm()

    async def land_drone(self):
        """Land the drone."""
        self.get_logger().info("Landing drone...")
        await self.drone.action.land()

    async def manual_control_drone(self, roll, pitch, throttle, yaw):
        """Send manual control command to the drone."""
        self.get_logger().info(
            f"Sending Manual Input: Roll={roll}, Pitch={pitch}, Throttle={throttle}, Yaw={yaw}"
        )
        await self.drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)

    def keyboard_callback(self, msg: String):
        """
        Callback from the keyboard publisher.
        Expects a comma-separated string of keys (e.g., "LEFT,UP,w").
        """
        keys = [k.strip() for k in msg.data.split(',') if k.strip()]
        # Default control values
        roll, pitch, throttle, yaw = 0.0, 0.0, 0.5, 0.0
        value = 0.5

        if "LEFT" in keys:
            pitch = -value
        elif "RIGHT" in keys:
            pitch = value
        if "UP" in keys:
            roll = value
        elif "DOWN" in keys:
            roll = -value
        if "w" in keys:
            throttle = 1.0
        elif "s" in keys:
            throttle = 0.0
        if "a" in keys:
            yaw = -value
        elif "d" in keys:
            yaw = value

        # Schedule specific commands
        if "r" in keys:
            asyncio.run_coroutine_threadsafe(self.arm_drone(), self.async_loop)
        elif "l" in keys:
            asyncio.run_coroutine_threadsafe(self.land_drone(), self.async_loop)
        else:
            asyncio.run_coroutine_threadsafe(self.manual_control_drone(roll, pitch, throttle, yaw), self.async_loop)

def main(args=None):
    rclpy.init(args=args)
    # Create a new asyncio event loop
    async_loop = asyncio.new_event_loop()

    # Run the asyncio loop in a separate thread
    thread = threading.Thread(target=lambda: async_loop.run_forever(), daemon=True)
    thread.start()

    node = MavlinkManualControlNode(async_loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MAVLink Manual Control Node Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Stop the asyncio loop and join the thread
        async_loop.call_soon_threadsafe(async_loop.stop)
        thread.join()

if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import threading
from mavsdk import System
import KeyPressModule as kp

# Global control variables (default values)
roll = 0.0
pitch = 0.0
throttle = 0.5
yaw = 0.0
CONTROL_STEP = 0.5  # Change step value

class MavlinkManualControlNode(Node):
    def __init__(self, async_loop):
        super().__init__('mavlink_manual_control_node')
        self.async_loop = async_loop

        # Initialize MAVSDK system
        self.drone = System()

        # Inform that the node is starting
        self.get_logger().info("Initializing MAVLink Manual Control Node...")

        # Schedule the drone connection and control tasks on the asyncio loop.
        asyncio.run_coroutine_threadsafe(self.run_drone(), self.async_loop)

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

        # Start keyboard reading and manual control loops concurrently.
        asyncio.create_task(self.get_keyboard_input())
        asyncio.create_task(self.manual_control_loop())

    async def get_keyboard_input(self):
        """
        Read key presses using KeyPressModule (kp) and update the control variables.
        Also, issue arm or land commands when "r" or "l" is pressed.
        """
        global roll, pitch, throttle, yaw
        kp.init()  # Initialize the key module (e.g., create a pygame window)
        while rclpy.ok():
            # Reset control values every loop
            roll, pitch, throttle, yaw = 0.0, 0.0, 0.5, 0.0
            value = CONTROL_STEP

            # Check keys and update control values
            if kp.getKey("LEFT"):
                pitch = -value
            elif kp.getKey("RIGHT"):
                pitch = value

            if kp.getKey("UP"):
                roll = value
            elif kp.getKey("DOWN"):
                roll = -value

            if kp.getKey("w"):
                throttle = 1.0
            elif kp.getKey("s"):
                throttle = 0.0

            if kp.getKey("a"):
                yaw = -value
            elif kp.getKey("d"):
                yaw = value

            # For simplicity, always issue arm when "r" is pressed, and land when "l" is pressed.
            # (PX4’s failsafe might disarm if other health checks fail.)
            if kp.getKey("r"):
                self.get_logger().info("Arm command triggered via keyboard.")
                await self.drone.action.arm()
            if kp.getKey("l"):
                self.get_logger().info("Land command triggered via keyboard.")
                await self.drone.action.land()
            if kp.getKey("i"):
                # Launch a flight mode printing task
                asyncio.create_task(self.print_flight_mode())

            await asyncio.sleep(0.1)

    async def print_flight_mode(self):
        async for flight_mode in self.drone.telemetry.flight_mode():
            self.get_logger().info(f"Flight Mode: {flight_mode}")
            break

    async def manual_control_loop(self):
        """
        Continuously send manual control input (roll, pitch, throttle, yaw) to the drone.
        """
        global roll, pitch, throttle, yaw
        while rclpy.ok():
            self.get_logger().info(
                f"Manual Input: Roll={roll:.2f}, Pitch={pitch:.2f}, Throttle={throttle:.2f}, Yaw={yaw:.2f}"
            )
            await self.drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
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

