# Sena Berra Soydugan, Orkan Çağan Dilber

import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, MissionProgress
from mavsdk.offboard import Attitude, OffboardError
import rospy

from drone_commands.action_command import Connect, Takeoff

async def main():

    drone = System()
    
    await Connect(drone, 14540)
    
    print("Clearing any existing missions...")
    await drone.mission.clear_mission()

    # Get the current position to calculate target altitude
    print("Fetching current position...")
    async for position in drone.telemetry.position():
        current_position = position
        print(f"Current position: {current_position.latitude_deg}, {current_position.longitude_deg}")
        break

    await drone.action.set_maximum_speed(25)
    # Define mission items
    mission_items = [
        MissionItem(
            47.397742,
            8.5475934,
            20,  # Altitude for takeoff
            20,  # Speed
            True,  # is_fly_through
            float('nan'),  # gimbal pitch
            float('nan'),  # gimbal yaw
            MissionItem.CameraAction.NONE,  # camera action
            float('nan'),  # loiter time
            float('nan'),  # camera photo interval
            float('nan'),  # acceptance radius
            float('nan'),  # yaw angle
            float('nan'),  # landing radius
            MissionItem.VehicleAction.NONE  # vehicle action
        ),
        MissionItem(
            47.397742,
            8.5495934,
            40,  # Altitude for waypoint
            10,  # Speed
            True,  # is_fly_through
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE
        ),
        MissionItem(
            47.3977418, # BUDUR
            8.54572,
            40,  # Altitude for waypoint
            10,  # Speed
            True,  # is_fly_through
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE
        ),
        MissionItem(
            47.397742,
            8.54382,
            20,  # Altitude set to 0 for landing
            10,  # Speed
            True,  # is_fly_through
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE 
        ),
            MissionItem(
            47.397742,
            8.54222,
            0,  # Altitude set to 0 for landing
            10,  # Speed
            True,  # is_fly_through30
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.LAND  # vehicle action
        )
    ]
    
    mission_plan = MissionPlan(mission_items)
    
    print("-- Uploading mission")
    try:
        await drone.mission.upload_mission(mission_plan)
        print("-- Mission uploaded successfully")
    except Exception as e:
        print(f"Failed to upload mission: {e}")
        return
    
    await Takeoff(drone)

    print("-- Starting mission")
    try:
        await drone.mission.start_mission()
        print("-- Mission started")
    except Exception as e:
        print(f"Failed to start mission: {e}")

    # 3. waypoint tamamlandıktan sonra offboard flight moda geçsin
    async for mission_progress in drone.mission.mission_progress():
        if mission_progress.current == len(mission_items) - 2:
            print("Mission completed, transitioning to offboard mode")
            break
    
    rospy.loginfo("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, -45.0, 0.0, 0.15))
    
    rospy.loginfo("-- Starting offboard")
    
    try:
        await drone.offboard.start()
    except OffboardError as error:
        rospy.logerr(f"Starting offboard mode failed with error code: {error._result.result}")
        await drone.action.disarm()
        return
    
    position = await drone.telemetry.position().__aiter__().__anext__()

    while position.relative_altitude_m > 15:
        await asyncio.sleep(0.1)
        position = await drone.telemetry.position().__aiter__().__anext__()
    
    await drone.offboard.set_attitude(Attitude(0.0, 15.0, 0.0, 0.15))
    await asyncio.sleep(3)
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.15))

    try:
        await drone.mission.start_mission()
        print("-- Mission started")
    except Exception as e:
        print(f"Failed to start mission: {e}")


if __name__ == "__main__":
    asyncio.run(main())
