import asyncio
from mavsdk import System
from mavsdk.offboard import Attitude, OffboardError


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14541")

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Roll 0 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 40.0, 0.0, 0.6))
    await asyncio.sleep(5)

    print("-- Roll 0 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(5)

    await drone.action.return_to_launch()
    print("--- Landing")
    await asyncio.sleep(5)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
