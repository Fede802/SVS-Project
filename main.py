import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility
import carla_utility

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()
spectator = world.get_spectator()

#TTC = (distance / relative_velocity) if relative_velocity != 0 else 9999
# Simple callback function to print the number of detections
def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
    print()
    #for m in data:
     #   debug_utility.draw_radar_point(radar, m)

pygame.init()
pygame.display.set_mode((400, 300))

for v in world.get_actors().filter('vehicle.*'):
    v.destroy()

#ego_vehicle = carla_utility.spawn_vehicle(world=world, vehicle_index=15, transform=carla.Transform(rotation = carla.Rotation(yaw=45)))
ego_vehicle = carla_utility.spawn_vehicle(world=world, vehicle_index=18)
other_vehicle = carla_utility.spawn_vehicle(world=world, transform=carla.Transform(carla.Location(x=50)))

radar = carla_utility.spawn_radar(world, ego_vehicle, range=100)
radar.listen(lambda data: handle_measurement(data, radar))

#vehicle.set_autopilot(True)
running = True
moving_forward = False
#carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())

try:
    while running:
        current_location = ego_vehicle.get_location()
        carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())
        #debug_utility.draw_radar_bounding_box(radar)
        #time.sleep(0.001)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                key = event.key
                if key == pygame.QUIT or key == pygame.K_ESCAPE:
                    running = False
                elif key == pygame.K_w:
                    current_location.x += 2
                    #world.tick()
                    moving_forward = True
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                elif key == pygame.K_s:
                    current_location.x -= 2
                    ego_vehicle.apply_control(carla.VehicleControl(brake=0.1))
                    #world.tick()
                    moving_forward = False

        
        
        world.tick()
        pygame.display.flip()
        #print("Actor transform: ", ego_vehicle.get_transform())
        #print("Actor forward vector (direction):", ego_vehicle.get_transform().get_forward_vector())
        #print("Actor velocity: ", ego_vehicle.get_velocity(),", ",ego_vehicle.get_velocity().length(),"m/s, ",ego_vehicle.get_velocity().length()*3.6,"km/h")
        
except KeyboardInterrupt:
    pass 
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    radar.destroy()
    ego_vehicle.destroy()
    other_vehicle.destroy()     