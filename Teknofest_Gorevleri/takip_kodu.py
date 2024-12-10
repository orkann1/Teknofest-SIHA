"""Copyright (C) 17/07/2024 Orkan Çağan Dilber

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of Orkan Çağan Dilber shall not be used in advertising or otherwise to promote the sale, use or other dealings in this Software without prior written authorization from Orkan Çağan Dilber."""

import asyncio
import rospy
from mavsdk import System
from mavsdk.offboard import Attitude, OffboardError
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import NavSatFix
import time

from drone_commands.action_command import Connect, Takeoff
from drone_commands.tracking_command import TrackingGPS, TrackingCamera
from drone_commands.telemetry_command import AltitudeRelative

class Tracking:
    def __init__(self):
        self.current_x = 0
        self.current_y = 0
        
        self.size_x = 0.0
        self.size_y = 0.0

        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.target_altitude = 0.0

        self.onceki_size_x = 0.0
        self.onceki_size_y = 0.0

        self.zamanlayici = 0.0

        # ROS Subscriber
        self.center_sub = rospy.Subscriber('/center_coordinates', Int32MultiArray, self.center_callback)
        self.size_y_sub = rospy.Subscriber('/size', Float32MultiArray, self.size_callback)
        self.gps_sub = rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, self.gps_callback)

    def center_callback(self, msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
    
    def size_callback(self, msg):
        self.size_x = msg.data[0]
        self.size_y = msg.data[1]

    def gps_callback(self, msg):
        self.target_latitude = msg.latitude
        self.target_longitude = msg.longitude
        self.target_altitude = msg.altitude

async def run():
    rospy.init_node('track_controller_node', anonymous=True)
    drone = System()

    await Connect(drone, 14540)
    await Takeoff(drone)
    
    track = Tracking()

    while not rospy.is_shutdown():
        rospy.loginfo("Siha GPS konumuna yönlendiriliyor.")
        track.zamanlayici = time.time()
        while 1 >= time.time() - track.zamanlayici :
            await TrackingGPS(drone, track.target_latitude, track.target_longitude,
                    track.target_altitude)
            if track.size_x == track.onceki_size_x and track.size_y == track.onceki_size_y:
                track.zamanlayici = time.time()
            
            track.onceki_size_x = track.size_x
            track.onceki_size_y = track.size_y

        rospy.loginfo("Siha görüntüde görüldü.")

        rospy.loginfo("-- Setting initial setpoint")
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.25))

        rospy.loginfo("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            rospy.logerr(f"Starting offboard mode failed with error code: {error._result.result}")
            await drone.action.land()
            await asyncio.sleep(15)
            await drone.action.disarm()
            return

        track.zamanlayici = time.time()
        while 1 >= time.time() - track.zamanlayici:
            altitude = await AltitudeRelative(drone)
            roll, pitch, thrust = TrackingCamera(track.current_x, track.current_y, track.size_y, altitude)

            if track.size_x != track.onceki_size_x and track.size_y != track.onceki_size_y:
                 track.zamanlayici = time.time()

            track.size_x = track.onceki_size_x 
            track.size_y = track.onceki_size_y

            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Thrust: {thrust:.2f}")
            
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
