import carla, time, pygame, cv2, debug_utility, carla_utility
import numpy as np
from server import start_servers, send_data, close_servers
from collections import deque
from pid_controller import PIDController
# x=2398.767090, y=5822.232910, z=177.039658

send_info = True
show_in_carla = False
show_in_camera = True
running = True
pid_control = False
update_frequency = 0.2 #seconds

radar_detection_h_radius = 1
radar_detection_v_radius = 0.8
max_target_velocity = 130
radar_range_offset = 20
radar_range = carla_utility.compute_security_distance(max_target_velocity) + radar_range_offset
target_velocity = 90
min_distance_offset = 7
min_permitted_distance = min_distance_offset

if send_info:
    start_servers()

client = carla.Client('localhost', 2000)
client.set_timeout(100.0)
spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=170), carla.Rotation(yaw = -88.2))
world = client.get_world()
spectator = world.get_spectator()

carla_utility.destroy_all_vehicle_and_sensors(world) #to avoid spawning bugs

video_output = np.zeros((600, 800, 4), dtype=np.uint8)
def camera_callback(image):
    global video_output
    video_output = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

def radar_callback(data: carla.RadarMeasurement, radar: carla.Actor):
    # print("Radar data received")
    global min_ttc, min_depth
    min_ttc = min_depth = float('inf')
    for detection, i in zip(data, range(len(data))):
        absolute_speed = abs(detection.velocity)
        # print(detection.velocity)
        if debug_utility.evaluate_point(radar, detection, radar_detection_h_radius, radar_detection_v_radius):
            # debug_utility.draw_radar_point(radar, detection)
            if absolute_speed != 0:
                ttc = detection.depth / absolute_speed
                if ttc < min_ttc:
                    min_ttc = ttc
            if detection.depth < min_depth:
                min_depth = detection.depth

pygame.init()
pygame.display.set_mode((400, 300))

ego_vehicle = carla_utility.spawn_vehicle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=spawn_point)
radar = carla_utility.spawn_radar(world, ego_vehicle, range=radar_range)
radar.listen(lambda data: radar_callback(data, radar))
carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())

other_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(world, ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=600)

if show_in_camera:
    camera = carla_utility.spawn_camera(world=world, attach_to=ego_vehicle, transform=carla.Transform(carla.Location(x=-6, z=5), carla.Rotation(pitch=-30)))
    camera.listen(lambda image: camera_callback(image))
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

pid_controller = PIDController()
lastUpdate = 0

try:
    while running:
        control = carla.VehicleControl()
        target_control = carla.VehicleControl()
        #other_vehicle.apply_control(carla.VehicleControl(throttle=0.3))
        
        # debug_utility.draw_radar_bounding_range(radar)
        # debug_utility.draw_radar_point_cloud_range(radar, radar_detection_h_radius, radar_detection_v_radius)
        
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
                    running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            control.throttle = 0.5
            # target_control.throttle = 1.0
        if keys[pygame.K_s]:
            # control.throttle = 0.5
            target_control.brake = 1.0
        if keys[pygame.K_a]:
            # control.steer = -0.38
            target_control.steer = -0.38
        if keys[pygame.K_d]:
            # control.steer = 0.38
            target_control.steer = 0.38
        if keys[pygame.K_e]:
            control.brake = 0.5
        if keys[pygame.K_r]:
            cruise_control = True
        if keys[pygame.K_p]:
            cruise_control = False
        
        if pid_control:
            min_permitted_distance = carla_utility.compute_security_distance(ego_vehicle.get_velocity().length() * 3.6) + min_distance_offset
            distance_error = min_depth - min_permitted_distance
            if distance_error > 0:
                control.throttle = pid_controller.compute_pid_control_speed(target_velocity, ego_vehicle.get_velocity().length() * 3.6)
            else:
                control.brake = pid_controller.compute_pid_control_distance(min_permitted_distance, min_depth)

        ego_vehicle.apply_control(control)
        other_vehicle.apply_control(target_control)
        print(spectator.get_transform())
        if show_in_carla:
            carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())
        if show_in_camera:
            cv2.imshow('RGB Camera', video_output)
        pygame.display.flip()
        if(time.time() - lastUpdate > update_frequency):
            if send_info:
                send_data({"velocity": ego_vehicle.get_velocity().length()*3.6, "acceleration": ego_vehicle.get_acceleration().length()})
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
