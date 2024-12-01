import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility
import carla_utility
import plot_utility

client = carla.Client('localhost', 2000)
print(client.get_available_maps())
client.load_world("Town10HD")
client.set_timeout(50.0)

world = client.get_world()
spectator = world.get_spectator()

spectator.set_transform(carla.Transform(carla.Location(x=-100, y=27, z=10), carla.Rotation(0,0,0)))

def handle_measurement(data: carla.RadarMeasurement, radar):
    for detection in data:
        #print(detection.velocity, detection.azimuth * 180 /math.pi, " ", detection.altitude * 180 /math.pi, detection.depth)
        
        if debug_utility.evaluate_point(radar, detection):
            debug_utility.draw_radar_point(radar, detection)
        #time.sleep(5)



for v in world.get_actors().filter('vehicle.*'):
    v.destroy()
ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=-60, y=27, z=5), carla.Rotation(yaw=180)))
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '30')
radar_bp.set_attribute('vertical_fov', '30')
radar_bp.set_attribute('range', str(50))
radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(z = 2)), ego_vehicle)
ego_vehicle2 = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=-70, y=27, z=5), carla.Rotation(yaw=180)))
time.sleep(2)
radar.listen(lambda data: handle_measurement(data, radar))
debug_utility.draw_radar_bounding_box(radar)

while True:
    continue