import asyncio
from mavsdk.action import OrbitYawBehavior 

# Sena Berra Soydugan
async def TrackingGPS(drone, target_latitude, target_longitude, target_altitude):
        yaw_behavior = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await drone.action.do_orbit(
                1,
                2,
                yaw_behavior,
                target_latitude,
                target_longitude,
                target_altitude-48,
                )
        await asyncio.sleep(0.1)

# Orkan Çağan Dilber
def TrackingCamera(current_x, current_y, size_y, altitude):
        # For X-axis
        kp = 60
        ki = 8
        kd = 6
        integral = 0
        derivative = 0
        last_error = 0
        normal_error = 0

        setpoint = 640

        error = int(current_x - setpoint)
        normal_error = error/setpoint
        integral = float(integral + normal_error)
        derivative = normal_error - last_error
        last_error = normal_error
        pid = (kp * normal_error + ki * integral + kd * derivative)

        # For Y-axis
        kp2 = 15
        ki2 = 5
        kd2 = 3
        integral2 = 0
        derivative2 = 0
        last_error2 = 0
        normal_error2 = 0

        setpoint2 = 360

        error2 = int(current_y - setpoint2)
        normal_error2 = error2/setpoint2
        integral2 = float(integral2+ normal_error2)
        derivative2 = normal_error2 - last_error2
        last_error2 = normal_error2
        pid2 = -(kp2 * normal_error2 + ki2 * integral2 + kd2 * derivative2)

        # For Thrust
        kp3 = 0.1
        ki3 = 0.01
        kd3 = 0.1
        integral3 = 0
        derivative3 = 0
        last_error3 = 0
        normal_error3 = 0

        setpoint3 = 42

        error3 = int(size_y - setpoint3)
        normal_error3 = error3/setpoint3
        integral3 = float(integral3+ normal_error3)
        derivative3 = normal_error3 - last_error3
        last_error3 = normal_error3
        pid3 = 0.25 - (kp3 * normal_error3 + ki3 * integral3 + kd3 * derivative3)
        
        pid = max(min(pid, 60), -60) # Max Roll Angle 60 Degrees

        if 5 < altitude < 115:
            pid2 = max(min(pid2, 15), -15) # Max Pitch Angle 30 Degrees
        else:
            pid2 = 0
            
        pid3 = max(min(pid3, 1), 0.10)

        return pid, pid2, pid3
