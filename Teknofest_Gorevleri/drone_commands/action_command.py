# Orkan Çağan Dilber

import asyncio
from mavsdk.action import OrbitYawBehavior

async def Connect(drone, UDP):
    await drone.connect(system_address=f"udp://:{UDP}")    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break


async def Takeoff(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Arming")
            await drone.action.arm()

            print("--- Taking Off")
            await drone.action.takeoff()
            await asyncio.sleep(4)
        break

async def Orbit(drone, radius_m, latitude, longitude, altitude):
    position = await drone.telemetry.position().__aiter__().__anext__()
    orbit_height = position.absolute_altitude_m - position.relative_altitude_m + altitude
    yaw_behavior = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER

    print(f'Do orbit at {altitude}m height from the ground')
    await drone.action.do_orbit(radius_m = radius_m,
                                velocity_ms = 2,
                                yaw_behavior = yaw_behavior,
                                latitude_deg = latitude,
                                longitude_deg = longitude,
                                absolute_altitude_m=orbit_height)
