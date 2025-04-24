import asyncio
import random
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from mavsdk.telemetry import LandedState
import KeyPressModule as kp

# Test set of manual inputs. Format: [north, east, down, yaw]

kp.init()
drone = System()

north, east, down, yaw = 0, 0, 0, 0
async def getKeyboardInput(my_drone):
    global north, east, down, yaw
    while True:
        # Reset values for this iteration
        north_change, east_change, down_change, yaw_change = 0, 0, 0, 0
        value = 0.1
        
        # Horizontal movement (east/west)
        if kp.getKey("LEFT"):
            east_change = -value
        elif kp.getKey("RIGHT"):
            east_change = value
            
        # Forward/backward movement (north/south)
        if kp.getKey("UP"):
            north_change = value
        elif kp.getKey("DOWN"):
            north_change = -value
            
        # Altitude control (down is positive in NED)
        if kp.getKey("w"):
            down_change = -value  # Move up
        elif kp.getKey("s"):
            down_change = value   # Move down
            
        # Rotation control (yaw)
        if kp.getKey("a"):
            yaw_change = -2
        elif kp.getKey("d"):
            yaw_change = 2
            
        # Update position values
        north += north_change
        east += east_change
        down += down_change
        yaw += yaw_change
        
        # Special commands
        if kp.getKey("i"):
            asyncio.ensure_future(print_flight_mode(my_drone))
        elif kp.getKey("r"):
            # Try to arm the drone
            print("Arming drone...")
            try:
                await my_drone.action.arm()
                print("Drone armed successfully!")
            except Exception as e:
                print(f"Arming failed: {e}")
        elif kp.getKey("l"):
            # Try to land the drone
            print("Landing drone...")
            try:
                await my_drone.action.land()
                north, east, down, yaw = 0, 0, 0, 0
                print("Landing command sent!")
            except Exception as e:
                print(f"Landing failed: {e}")
        
        await asyncio.sleep(0.1)

async def print_flight_mode(my_drone):
    async for flight_mode in my_drone.telemetry.flight_mode():
        print("FlightMode:", flight_mode)
        # return flight_mode

async def manual_control_drone(my_drone):
    global north, east, down, yaw
    
    # Set the initial setpoint before starting offboard mode
    print("Setting initial setpoint...")
    await my_drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
    
    # Start offboard mode
    print("Starting offboard mode...")
    try:
        await my_drone.offboard.start()
        print("Offboard mode started!")
    except Exception as e:
        print(f"Starting offboard mode failed: {e}")
        print("Trying to arm the drone first...")
        try:
            await my_drone.action.arm()
            await asyncio.sleep(2)
            await my_drone.offboard.start()
            print("Offboard mode started after arming!")
        except Exception as e2:
            print(f"Arming and starting offboard mode failed: {e2}")
    
    # Now we can send position commands
    while True:
        print(f"Sending position: North={north}, East={east}, Down={down}, Yaw={yaw}")
        try:
            await my_drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
        except Exception as e:
            print(f"Failed to set position: {e}")
        await asyncio.sleep(0.2)

async def run_drone():
    asyncio.ensure_future(getKeyboardInput(drone))
    await drone.connect(system_address="udp://:14540")
    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    
    # Check and print the drone status to help diagnose issues
    print("Checking drone status...")
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        break
    
    # Check if drone is ready to arm
    print("Checking if drone is ready to arm...")
    async for is_armable in drone.telemetry.health_all_ok():
        print(f"Is drone ready to arm: {is_armable}")
        break
    
    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break
    asyncio.ensure_future(manual_control_drone(drone))


async def run():
    global north, east, down, yaw
    """Main function to connect to the drone and input manual controls"""
    await asyncio.gather(run_drone())

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
