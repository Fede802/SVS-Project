import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility
import carla_utility
import plot_utility

client = carla.Client('localhost', 2000)
print(client.get_available_maps())
#time.sleep(1000)
#3 bella dritta, 4 anche meglio


world = client.get_world()
spectator = world.get_spectator()

f = open("log.txt", "a")

#vPlot = plot_utility.RealTimePlotApp("Velocity")
#aPlot = plot_utility.RealTimePlotApp("Acceleration")

target_velocity = 70
def lidar_callback(data: carla.LidarMeasurement):
    print("-------------LIDAR MESUREMENT-------------")
    print(data)
    f.write("-------------LIDAR MESUREMENT-------------\n")
    f.write(str(data)+"\n")
    f.write(str(len(data))+"\n")
    for location in data:
        #f.write(str(location)+"\n")
        print(location)
    f.write("-----------END LIDAR MESUREMENT------------\n")    
    print("-----------END LIDAR MESUREMENT------------")    
#TTC = (distance / relative_velocity) if relative_velocity != 0 else 9999
# Simple callback function to print the number of detections
min_ttc = float('inf')
def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_distance
    min_ttc = float('inf')
    
    for detection, i in zip(data, range(len(data))):
        print(detection)
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
#ego_vehicle = carla_utility.spawn_vehicle(world=world)
#ego_vehicle = carla_utility.spawn_veichle_at(world=world, vehicle_index=15, spawn_point=carla.Transform(carla.Location(x=2.484849, y=-170.415253, z=2.900956)))
#ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=380.786957, y=31.491543, z=13.309415), carla.Rotation(yaw = 180)))
ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=408.760681, y=-71.501823, z=7.077427), carla.Rotation(yaw = 102)))
print(ego_vehicle.get_location())
#other_vehicle = carla_utility.spawn_vehicle(world=world, transform=carla.Transform(carla.Location(x=50)))
other_vehicle = carla_utility.spawn_veichle_bp_in_front_of(world, ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=10)

#radar = carla_utility.spawn_radar(world, ego_vehicle, range=54)
#radar.listen(lambda data: handle_measurement(data, radar))
lidar = carla_utility.spawn_lidar(world, ego_vehicle)
lidar.listen(lambda data: lidar_callback(data))

video_output = np.zeros((600, 800, 4), dtype=np.uint8)
def camera_callback(image):
    global video_output
    video_output = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))





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
    errore_vel = velocita_target - (velocita_corrente * 3.6)
    #print(errore_vel)

    errore_vel_perc = abs(errore_vel) * 100 / velocita_target
    
    # Inizializza throttle e brake
    throttle = 0.0
    brake = 0.0
    
    if min_ttc > 0 and min_ttc < float('inf'):
        print(min_ttc)
        brake = 1.0
    else:    
        if errore_vel > 0:
            throttle = 1.0 * errore_vel_perc
        else:
            brake = 1.0 * errore_vel_perc    
        # Accelerazione
     #   throttle = min(0.5 + k_p * errore_vel, 1.0)
    #elif errore_vel < 0:
        # Decelerazione
     #   brake = min(-k_p * errore_vel, 1.0)

    #if brake > 0:
     #   throttle = 0.0
    
    # TTC Safety Override
    #if ttc < 1.5:
     #   brake = 1.0
      #  throttle = 0.0
    
    return throttle, brake

carla_utility.move_spectator_to(world, ego_vehicle.get_transform())
show_in_carla = False
show_in_camera = True
running = True
cruise_control = False
lastUpdate = 0

if show_in_camera:
    camera = carla_utility.spawn_camera(world=world, attach_to=ego_vehicle, transform=carla.Transform(carla.Location(x=-6, z=5), carla.Rotation(pitch=-30)))
    camera.listen(lambda image: camera_callback(image))
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

try:
    while running:
        control = carla.VehicleControl()
        other_vehicle.apply_control(carla.VehicleControl(brake=0.5))
        #other_vehicle.apply_control(carla.VehicleControl(throttle=0.5))
        #control.throttle = 1.0

        #debug_utility.draw_radar_bounding_box(radar)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
                    running = False
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            control.throttle = 0.5
        if keys[pygame.K_s]:
            control.throttle = 0.5
            control.reverse = True
        if keys[pygame.K_a]:
            control.steer = -0.38
        if keys[pygame.K_d]:
            control.steer = 0.38
        if keys[pygame.K_e]:
            control.brake = 0.5
        if keys[pygame.K_r]:
            cruise_control = True
        if keys[pygame.K_p]:
            cruise_control = False 
        if cruise_control:              
            throttle, brake = compute_controls(ego_vehicle.get_velocity().length(), target_velocity, min_ttc)
            control = carla.VehicleControl(throttle=throttle, brake=brake)
        print(control)    
        ego_vehicle.apply_control(control)
        

        if show_in_carla:
            carla_utility.move_spectator_to(world, ego_vehicle.get_transform())
        if show_in_camera:
            cv2.imshow('RGB Camera', video_output)
        pygame.display.flip()
        #print("Actor transform: ", ego_vehicle.get_transform())
        #print("Actor forward vector (direction):", ego_vehicle.get_transform().get_forward_vector())
        #print("Actor control: ",ego_vehicle.get_control().throttle," ", ego_vehicle.get_control().brake," Actor velocity: ", ego_vehicle.get_velocity(),", ",ego_vehicle.get_velocity().length(),"m/s, ",ego_vehicle.get_velocity().length()*3.6,"km/h")
        
        if(time.time() - lastUpdate > 0.2):
            #vPlot.add_value(ego_vehicle.get_velocity().length()*3.6)
            #aPlot.add_value(ego_vehicle.get_acceleration().length())
            lastUpdate = time.time()
        world.tick()
except KeyboardInterrupt:
    pass 
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    #radar.stop()
    #radar.destroy()
    f.close()
    camera.destroy()
    lidar.destroy()
    ego_vehicle.destroy()
    other_vehicle.destroy()     