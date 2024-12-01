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

# target_velocity = 50






# min_ttc = float('inf')

# def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
#     global min_ttc, min_distance
#     min_ttc = float('inf')
#     for detection, i in zip(data, range(len(data))):
#         absolute_speed = abs(detection.velocity)
#         if debug_utility.evaluate_point(radar, detection):
#             debug_utility.draw_radar_point(radar, detection)
#             if absolute_speed != 0:
#                 #debug_utility.draw_radar_point(radar, detection)
#                 ttc = detection.depth / absolute_speed
#                 if ttc < min_ttc:
#                     min_ttc = ttc

# def compute_controls(velocita_corrente, velocita_target):
#     errore_vel = velocita_target - (velocita_corrente * 3.6)

#     errore_vel_perc = abs(errore_vel) * 100 / velocita_target
    
#     throttle = 0.0
#     brake = 0.0

#     if min_ttc > 0 and min_ttc < float('inf'):
#          brake = 1.0
#     else:    
#         if errore_vel > 0:
#             throttle = 1.0 * errore_vel_perc
#         else:
#             brake = 1.0 * errore_vel_perc 
    
#     return throttle, brake

for v in world.get_actors().filter('vehicle.*'):
    v.destroy()
for v in world.get_actors().filter('sensor.*'):
    v.destroy()
ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=-60, y=27, z=5), carla.Rotation(yaw=0)))
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '30')
radar_bp.set_attribute('vertical_fov', '30')
radar_bp.set_attribute('range', str(100))
radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(z = 2)), ego_vehicle)
#ego_vehicle2 = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=-70, y=27, z=5), carla.Rotation(yaw=180)))
time.sleep(2)
print(radar.get_transform())
# radar.listen(lambda data: handle_measurement(data, radar))
# debug_utility.draw_radar_bounding_box(radar)

while True:
    debug_utility.draw_radar_bounding_range(radar)
    # throttle, brake = compute_controls(ego_vehicle.get_velocity().length(), target_velocity)
    # control = carla.VehicleControl(throttle=throttle, brake=brake)
    #ego_vehicle.apply_control(control)
    continue