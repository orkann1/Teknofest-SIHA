# Orkan Çağan Dilber

async def Health(drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
        break    

async def AltitudeRelative(drone):
    async for position in drone.telemetry.position():
        return position.relative_altitude_m

async def AltitudeAbsolute(drone):
    async for position in drone.telemetry.position():
        return position.absolute_altitude_m

async def Speed(drone):
    async for metrics in drone.telemetry.fixedwing_metrics():
        return metrics.airspeed_m_s