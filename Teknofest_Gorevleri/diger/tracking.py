"""Copyright (C) 17/07/2024 Orkan Çağan Dilber

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of Orkan Çağan Dilber shall not be used in advertising or otherwise to promote the sale, use or other dealings in this Software without prior written authorization from Orkan Çağan Dilber."""

import asyncio
import rospy
from mavsdk import System
from mavsdk.action import OrbitYawBehavior
from mavsdk.offboard import Attitude, OffboardError
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class PIDController:
    def __init__(self):
        self.current_x = 0
        self.current_y = 0
        self.target_x = 640
        self.target_y = 360
        self.size_x = 0.0
        self.size_y = 0.0
        self.subscribed = False

        self.center_sub = rospy.Subscriber('/center_coordinates', Int32MultiArray, self.center_callback)
        self.size_y_sub = rospy.Subscriber('/size', Float32MultiArray, self.size_callback)
        self.status_pub = rospy.Publisher('/plane0/status', Float32MultiArray, queue_size=50)

    def center_callback(self, msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.check_subscription()
    
    def size_callback(self, msg):
        self.size_x = msg.data[0]
        self.size_y = msg.data[1]

    def check_subscription(self):
        if not self.subscribed:
            self.subscribed = True
        # rospy.loginfo(f"Updated current position: x={self.current_x}, y={self.current_y}")

    async def get_altitude(self, drone):
        async for position in drone.telemetry.position():
            return position.relative_altitude_m
    
    async def get_speed(self, drone):
        async for metrics in drone.telemetry.fixedwing_metrics():
            return float(metrics.airspeed_m_s)

    def compute_control(self, altitude):
        if not self.subscribed:
            return None, None, None
        
        output_roll = max(min((self.current_x - self.target_x) / 10.67, 60), -60) # Max Roll Angle 60 Degrees
        if altitude <= 15:
            output_pitch = 10
        elif altitude >= 105:
            output_pitch = -10
        else:
            output_pitch = max(min((self.target_y - self.current_y) / 24, 15), -15) - 2.33 # Max Pitch Angle 30 Degrees
            
        output_thrust = max(min(10 / (self.size_y), 0.28), 0.18)

        return output_roll, output_pitch, output_thrust
    
    def publish_status(self, pitch, roll, thrust, altitude, speed):
        status_msg = Float32MultiArray()
        status_msg.data = [pitch, roll, thrust, altitude, speed]
        self.status_pub.publish(status_msg)

        
async def run():
    rospy.init_node('pid_controller_node', anonymous=True)
    drone = System()

    try:
        await drone.connect(system_address="udp://:14540")
    except Exception as e:
        rospy.logerr(f"Failed to connect to the drone: {e}")
        return

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Arming")
            await drone.action.arm()

            print("--- Taking Off")
            await drone.action.takeoff()
            await asyncio.sleep(10)
        break
    
    position = await drone.telemetry.position().__aiter__().__anext__()
    orbit_height = position.absolute_altitude_m - position.relative_altitude_m + 30
    yaw_behavior = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER

    print('Do orbit at 30m height from the ground')
    await drone.action.do_orbit(radius_m=100,
                                velocity_ms=2,
                                yaw_behavior=yaw_behavior,
                                latitude_deg=47.398036222362471,
                                longitude_deg=8.5350146439425509,
                                absolute_altitude_m=orbit_height)
    await asyncio.sleep(1)        
    
    print("Waiting to Detect")
    pid_controller = PIDController()

    while not pid_controller.subscribed:
        await asyncio.sleep(1)

    rospy.loginfo("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.25))

    rospy.loginfo("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        rospy.logerr(f"Starting offboard mode failed with error code: {error._result.result}")
        await drone.action.disarm()
        return

    while not rospy.is_shutdown():
        altitude = await pid_controller.get_altitude(drone)
        speed = await pid_controller.get_speed(drone)
        roll, pitch, thrust = pid_controller.compute_control(altitude)

        pid_controller.publish_status(roll, pitch, thrust, altitude, speed)

        try:
            await drone.offboard.set_attitude(Attitude(roll, pitch, 0.0, thrust))
        except OffboardError as error:
            rospy.logerr(f"Setting attitude failed with error code: {error._result.result}")
            await drone.action.disarm()
            return
        await asyncio.sleep(0.1)

    rospy.loginfo("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        rospy.logerr(f"Stopping offboard mode failed with error code: {error._result.result}")
    finally:
        await drone.action.disarm()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
