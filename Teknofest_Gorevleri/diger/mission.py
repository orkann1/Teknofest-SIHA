#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    
    await drone.mission.clear_mission()
    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_items = []
    mission_items.append(MissionItem(47.398036222362471,
                                     8.5450146439425509,
                                     25,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.TAKEOFF))
    mission_items.append(MissionItem(47.397825620791885,
                                     8.5450092830163271,
                                     25,
                                     30,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(47.387825620791885,
                                     8.5350092830163271,
                                     25,
                                     30,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(47.377825620791885,
                                     8.5350092830163271,
                                     25,
                                     30,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))
    
    mission_items.append(MissionItem(47.398039859999997,
                                     8.5455725400000002,
                                     0,  # Altitude set to 0 for landing
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.LAND))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    try:
        await drone.mission.upload_mission(mission_plan)
        print("-- Mission uploaded successfully")
    except Exception as e:
        print(f"Failed to upload mission: {e}")
        return

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    try:
        await drone.action.arm()
        print("-- Drone armed successfully")
    except Exception as e:
        print(f"Failed to arm drone: {e}")
        return

    async for state in drone.telemetry.armed():
        print(f"Armed status: {state}")
        if state:
            break
        
    await drone.action.set_maximum_speed(30)

    print("-- Starting mission")
    try:
        await drone.mission.start_mission()
        print("-- Mission started")
    except Exception as e:
        print(f"Failed to start mission: {e}")

    await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")

async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and returns after landing """
    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
