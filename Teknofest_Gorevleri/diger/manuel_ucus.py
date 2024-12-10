import asyncio
from mavsdk import System
from mavsdk.offboard import Attitude, OffboardError
import rospy


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14541")

    print("-- Arming")
    await drone.action.arm()
    
    print("-- Taking Off")
    await drone.action.takeoff()

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.15))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    while not rospy.is_shutdown:
        choice = input()
        if choice == 'a':
            await drone.offboard.set_attitude(Attitude(-15.0, 0.0, 0.0, 0.15))
            await asyncio.sleep(0.1)

        elif choice == 'd':
            await drone.offboard.set_attitude(Attitude(15.0, 0.0, 0.0, 0.15))
            await asyncio.sleep(0.1)

        elif choice == 's':
            await drone.offboard.set_attitude(Attitude(0.0, -15.0, 0.0, 0.15))
            await asyncio.sleep(0.1)

        elif choice == 'w':
            await drone.offboard.set_attitude(Attitude(0.0, 15.0, 0.0, 0.15))
            await asyncio.sleep(0.1)
        
        elif choice == 'x':
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.15))
            await asyncio.sleep(0.1)

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
