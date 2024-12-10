# Sena Berra Soydugan

import rospy
import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Circle, FenceType, GeofenceData, Polygon
from drone_commands.action_command import Connect, Takeoff, Orbit
from math import radians, cos, sin, sqrt, atan2

async def Terrarian(drone):
    print("Fetching home location coordinates...")
    async for terrain_info in drone.telemetry.home():
        latitude = terrain_info.latitude_deg
        longitude = terrain_info.longitude_deg
        print(f"Home location: ({latitude}, {longitude})")
        return latitude, longitude

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def point_in_polygon(lat, lon, poly):
    num = len(poly)
    j = num - 1
    inside = False
    for i in range(num):
        if ((poly[i][1] > lon) != (poly[j][1] > lon)) and \
                (lat < (poly[j][0] - poly[i][0]) * (lon - poly[i][1]) / (poly[j][1] - poly[i][1]) + poly[i][0]):
            inside = not inside
        j = i
    return inside

async def find_nearest_safe_location(current_lat, current_lon, safe_locations):
    min_distance = float('inf')
    nearest_location = None
    for location in safe_locations:
        distance = calculate_distance(current_lat, current_lon, location['latitude'], location['longitude'])
        if distance < min_distance:
            min_distance = distance
            nearest_location = location
    return nearest_location

async def check_geofence(drone, circles, polygon_points, safe_locations):
    async for position in drone.telemetry.position():
        lat = position.latitude_deg
        lon = position.longitude_deg
        
        outside_all = True
        for center_lat, center_lon, radius in circles:
            distance = calculate_distance(lat, lon, center_lat, center_lon)
            if distance <= radius:
                print("Your vehicle is within the geofence boundaries (circle).")
                outside_all = False
                nearest_safe_location = await find_nearest_safe_location(lat, lon, safe_locations)
                if nearest_safe_location:
                    print(f"Nearest safe location found: {nearest_safe_location['name']} at ({nearest_safe_location['latitude']}, {nearest_safe_location['longitude']})")
                    await Orbit(drone, 25, nearest_safe_location['latitude'], nearest_safe_location['longitude'], 25)            
                else:
                    print("No safe location found.")
                break

        if outside_all:
            polygon_coords = [(p.latitude_deg, p.longitude_deg) for p in polygon_points]
            if point_in_polygon(lat, lon, polygon_coords):
                print("Your vehicle is within the geofence boundaries (polygon).")
                outside_all = False

        if outside_all:
            print("Your vehicle is outside the geofence boundaries! Finding nearest safe location.")
            nearest_safe_location = await find_nearest_safe_location(lat, lon, safe_locations)
            if nearest_safe_location:
                print(f"Nearest safe location found: {nearest_safe_location['name']} at ({nearest_safe_location['latitude']}, {nearest_safe_location['longitude']})")
                await Orbit(drone, 25, nearest_safe_location['latitude'], nearest_safe_location['longitude'], 25)            
            else:
                print("No safe location found.")
        
        await asyncio.sleep(0.5)
        break

async def run():
    rospy.init_node('drone_geofence_node')
    
    drone = System()
    await Connect(drone, 14540)

    latitude, longitude = await Terrarian(drone)

    await asyncio.sleep(1)
    
    await drone.geofence.clear_geofence()
    print("Cleared existing geofences")

    p1 = Point(latitude - 0.0001, longitude - 0.0001)
    p2 = Point(latitude - 0.0007, longitude + 0.0020)

    circle1 = Circle(p1, 100.0, FenceType.EXCLUSION)
    circle2 = Circle(p2, 40.0, FenceType.EXCLUSION)

    p3 = Point(latitude - 0.0025, longitude - 0.006)
    p4 = Point(latitude - 0.0025, longitude + 0.006)
    p5 = Point(latitude + 0.0025, longitude + 0.006)
    p6 = Point(latitude + 0.0025, longitude - 0.006)

    polygon = Polygon([p3, p4, p5, p6], FenceType.INCLUSION)

    geofenceData = GeofenceData(polygons=[polygon], circles=[circle1, circle2])

    print("Uploading geofence...")
    try:
        await drone.geofence.upload_geofence(geofenceData)
        print("Geofence uploaded successfully!")
    except Exception as e:
        print(f"Failed to upload geofence: {e}")

    await Takeoff(drone)

    safe_locations = [
        {'name': 'Safe Location 1', 'latitude': latitude + 0.001, 'longitude': longitude + 0.001},
        {'name': 'Safe Location 2', 'latitude': latitude - 0.001, 'longitude': longitude - 0.001},
        # Add more safe locations as needed
    ]

    while not rospy.is_shutdown():
        await check_geofence(drone, [(p1.latitude_deg, p1.longitude_deg, circle1.radius), 
                                     (p2.latitude_deg, p2.longitude_deg, circle2.radius)], 
                            [p3, p4, p5, p6], safe_locations)

if __name__ == "__main__":
    asyncio.run(run())
