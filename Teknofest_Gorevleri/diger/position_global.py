import asyncio
import rospy
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionGlobalYaw)

async def run():
        drone = System()
        await drone.connect(system_address="udp://:14540")

        rospy.loginfo("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                rospy.loginfo("-- Connected to drone!")
                break

        rospy.loginfo("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                rospy.loginfo("-- Global position estimate OK")
                break

        target_latitude = 47.4084163
        target_longitude = 8.5554128
        target_altitude = 575.1094879880884

        rospy.loginfo("-- Arming")
        await drone.action.arm()

        rospy.loginfo("-- Taking Off")
        await drone.action.takeoff()
        await asyncio.sleep(10)

        rospy.loginfo("-- Setting initial setpoint")
        await drone.offboard.set_position_global(PositionGlobalYaw(47.3984163, 8.5454128, 568.1094879880884, 0.0, 1))
        rospy.loginfo("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            rospy.logerr(f"Starting offboard mode failed with error code: {error._result.result}")
            rospy.loginfo("-- Disarming")
            await drone.action.disarm()
            return

        await drone.offboard.set_position_global(PositionGlobalYaw(
                    target_latitude,
                    target_longitude,
                    target_altitude,
                    0.0,
                    1
                    )
        )

        rospy.loginfo("-- Returning")
        await drone.action.return_to_launch()

        rospy.loginfo("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            rospy.logerr(f"Stopping offboard mode failed with error code: {error._result.result}")
            rospy.loginfo("-- Attempting to land")
            await drone.action.land()

if __name__ == "__main__":
        rospy.init_node("drone_control_script")
        asyncio.run(run())