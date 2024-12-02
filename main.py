import carla, time, pygame, cv2, debug_utility, carla_utility
import numpy as np
from server import start_servers, send_data, close_servers

send_info = True
client = carla.Client('localhost', 2000)
if send_info:
    start_servers()
print(client.get_available_maps())
#3 bella dritta, 4 anche meglio
client.load_world('Town04')
client.set_timeout(100.0)

world = client.get_world()
spectator = world.get_spectator()

carla_utility.destroy_all_vehicle_and_sensors(world) #to avoid spawning bugs

radar_detection_h_radius = 1
radar_detection_v_radius = 1.3
target_velocity = 70
min_permitted_distance = 50
medium_permitted_distance = 100

def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_depth
    min_ttc = min_depth = float('inf')
    for detection, i in zip(data, range(len(data))):
        absolute_speed = abs(detection.velocity)
        if debug_utility.evaluate_point(radar, detection, radar_detection_h_radius, radar_detection_v_radius):
            debug_utility.draw_radar_point(radar, detection)
            if absolute_speed != 0:
                ttc = detection.depth / absolute_speed
                if ttc < min_ttc:
                    min_ttc = ttc
            elif detection.depth < min_permitted_distance:
                if detection.depth < min_depth:
                    min_depth = detection.depth

pygame.init()
pygame.display.set_mode((400, 300))

ego_vehicle = carla_utility.spawn_vehicle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=380.786957, y=31.491543, z=13.309415), carla.Rotation(yaw = 180)))
other_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(world, ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=40)

radar = carla_utility.spawn_radar(world, ego_vehicle, range=70)
radar.listen(lambda data: handle_measurement(data, radar))

video_output = np.zeros((600, 800, 4), dtype=np.uint8)
def camera_callback(image):
    global video_output
    video_output = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

def compute_controls(velocita_corrente, velocita_target):
    # se maggiore di 0 allora devo accellerare
    errore_vel = velocita_target - (velocita_corrente * 3.6)
    errore_vel_perc = abs(errore_vel) * 100 / velocita_target
    
    throttle = 0.0
    brake = 0.0
    
    if min_depth < min_permitted_distance:
        print("Too close, Emergency Brake!")
        brake = 1.0
    else:
        if min_depth < medium_permitted_distance:
            print("We are getting closer, little Brake!")
            brake = 0.5

    # if min_depth < min_permitted_distance or (min_ttc > 0 and min_ttc < float('inf')):
    #     print("Too close, Emergency Brake!")
    #     brake = 1.0
        else:    
            if errore_vel > 0:
                print("Ok we can throttle to reach target velocity")
                throttle = 1.0 * errore_vel_perc
            else:
                print("Little brake to mantein target velocity")
                brake = 1.0 * errore_vel_perc

    return throttle, brake

carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())
show_in_carla = False
show_in_camera = True
running = True
cruise_control = True
lastUpdate = 0

if show_in_camera:
    camera = carla_utility.spawn_camera(world=world, attach_to=ego_vehicle, transform=carla.Transform(carla.Location(x=-6, z=5), carla.Rotation(pitch=-30)))
    camera.listen(lambda image: camera_callback(image))
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

try:
    while running:
        control = carla.VehicleControl()
        other_vehicle.apply_control(carla.VehicleControl(throttle=0.3))
        
        debug_utility.draw_radar_bounding_range(radar)
        debug_utility.draw_radar_point_cloud_range(radar, radar_detection_h_radius, radar_detection_v_radius)
        
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
            throttle, brake = compute_controls(ego_vehicle.get_velocity().length(), target_velocity)
            control = carla.VehicleControl(throttle=throttle, brake=brake)
        ego_vehicle.apply_control(control)
        
        if show_in_carla:
            carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())
        if show_in_camera:
            cv2.imshow('RGB Camera', video_output)
        pygame.display.flip()
       
        if(time.time() - lastUpdate > 0.2):
            if send_info:
                send_data({"velocity": ego_vehicle.get_velocity().length(), "acceleration": ego_vehicle.get_acceleration().length()})
            lastUpdate = time.time()
        world.tick()
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    cv2.destroyAllWindows()  
    carla_utility.destroy_all_vehicle_and_sensors(world)  
    if send_info: 
        close_servers()
