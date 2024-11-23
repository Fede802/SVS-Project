import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility
import carla_utility

client = carla.Client('localhost', 2000)
print(client.get_available_maps())
#3 bella dritta, 4 anche meglio
client.load_world('Town04')
client.set_timeout(40.0)

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()
spectator = world.get_spectator()





min_ttc = float('inf')
target_velocity = 50

#TTC = (distance / relative_velocity) if relative_velocity != 0 else 9999
# Simple callback function to print the number of detections
def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_distance
    min_ttc = float('inf')

    for detection, i in zip(data, range(len(data))):
        absolute_speed = abs(detection.velocity)
        #debug_utility.draw_radar_point(radar, detection)
        # Calculate TTC
        if absolute_speed != 0:
            ttc = detection.depth / absolute_speed
            if ttc < min_ttc:
                min_ttc = ttc

pygame.init()
pygame.display.set_mode((400, 300))

for v in world.get_actors().filter('vehicle.*'):
    v.destroy()

#ego_vehicle = carla_utility.spawn_vehicle(world=world, vehicle_index=15, transform=carla.Transform(rotation = carla.Rotation(yaw=45)))
#ego_vehicle = carla_utility.spawn_vehicle(world=world, vehicle_index=15, spawn_index=3)
ego_vehicle = carla_utility.spawn_veichle_at(world=world, vehicle_index=15, spawn_point=carla.Transform(carla.Location(x=2.484849, y=-170.415253, z=2.900956)))
other_vehicle = carla_utility.spawn_vehicle(world=world, transform=carla.Transform(carla.Location(x=50)))

radar = carla_utility.spawn_radar(world, ego_vehicle)
radar.listen(lambda data: handle_measurement(data, radar))

#vehicle.set_autopilot(True)
running = True
moving_forward = False
#carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())

def compute_controls(velocita_corrente, velocita_target, ttc, k_p=0.05):
    """
    Calcola throttle e brake in base alla velocità corrente, velocità target e TTC.
    
    Args:
        velocita_corrente (float): Velocità attuale del veicolo in m/s.
        velocita_target (float): Velocità desiderata in m/s.
        ttc (float): Time to collision in secondi.
        k_p (float): Fattore di proporzionalità per il controllo.
        
    Returns:
        throttle (float): Valore tra 0.0 e 1.0.
        brake (float): Valore tra 0.0 e 1.0.
    """
    errore_vel = velocita_target - velocita_corrente
    
    # Inizializza throttle e brake
    throttle = 0.0
    brake = 0.0
    
    if errore_vel > 0:
        # Accelerazione
        throttle = min(0.5 + k_p * errore_vel, 1.0)
    elif errore_vel < 0:
        # Decelerazione
        brake = min(-k_p * errore_vel, 1.0)

    if brake > 0:
        throttle = 0.0
    
    # TTC Safety Override
    if ttc < 1.5:
        brake = 1.0
        throttle = 0.0
    
    return throttle, brake


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
        throttle, brake = compute_controls(ego_vehicle.get_velocity().length(), target_velocity, min_ttc)
        control = carla.VehicleControl(throttle=throttle, brake=brake)
        ego_vehicle.apply_control(control)
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
    radar.stop()
    radar.destroy()
    ego_vehicle.destroy()
    other_vehicle.destroy()     