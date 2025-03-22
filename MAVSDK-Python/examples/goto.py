#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

 #   async for h in drone.telemetry.home():
  #      home_location = { "lat": h.latitude_deg, "lon": h.longitude_deg}
   #     print("Home position", home_location)

    latitude = drone.telemetry.home.latitude_deg
    longitude = drone.telemetry.home.longitude_deg

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)
    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(latitude, longitude, flying_alt, 0) # Home position {'lat': 47.3968165, 'lon': 8.5497145}

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
