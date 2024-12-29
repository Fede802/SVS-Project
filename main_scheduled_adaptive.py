import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility 
from log_utility import Logger 
import numpy as np
from server import start_servers, send_data, close_servers
from pid_controller_scheduled_adaptive import PIDController
from manual_control import ControlInfo, DualControl, World
import plot_utility 

send_info = False
save_info = True
show_log = False
show_in_carla = False

pid_cc = False
update_frequency = 0.01 #seconds

radar_detection_h_radius = 1
radar_detection_v_radius = 0.8
max_target_velocity = 130
radar_range_offset = 20
radar_range = carla_utility.compute_security_distance(max_target_velocity) + radar_range_offset
target_velocity = 90
min_distance_offset = 7
min_permitted_distance = min_distance_offset
learning_rate = 0.003

if send_info:
    start_servers()

client = carla.Client('localhost', 2000)
client.set_timeout(30.0)

spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
pygame.init()
pygame.font.init()
# pygame.display.set_mode((400, 300))
display = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
#hud = hud.HUD(1280, 720)
pid_controller = PIDController(learning_rate, update_frequency, buffer_size=None) #add buffer_size = None to disable buffer
control_info = ControlInfo(pid_cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
world = World(client.get_world())
controller = DualControl(control_info)
# while True:
#     continue
# world = World(client.get_world())
# controller = DualControl(world, False, control_info)

clock = pygame.time.Clock()
# spectator = world.world.get_spectator()

# world.restart() #to avoid spawning bugs



def radar_callback(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_depth
    min_ttc = min_depth = float('inf')
    for detection, i in zip(data, range(len(data))):
        absolute_velocity = abs(detection.velocity)
        if debug_utility.evaluate_point(radar, detection, radar_detection_h_radius, radar_detection_v_radius):
            # debug_utility.draw_radar_point(radar, detection)
            if absolute_velocity != 0:
                ttc = detection.depth / absolute_velocity
                if ttc < min_ttc:
                    min_ttc = ttc
            if detection.depth < min_depth:
                min_depth = detection.depth



ego_vehicle = world.player
radar = carla_utility.spawn_radar(world.world, ego_vehicle, range=radar_range)
radar.listen(lambda data: radar_callback(data, radar))
# carla_utility.move_spectator_to(spectator, ego_vehicle.get_transform())

other_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(world.world, ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=100)




logger = Logger()
lastUpdate = 0

try:
    while control_info.running:
        # debug_utility.draw_radar_bounding_range(radar)
        # debug_utility.draw_radar_point_cloud_range(radar, radar_detection_h_radius, radar_detection_v_radius)
        if(time.time() - lastUpdate > update_frequency):
            if control_info.pid_cc:
                pid_controller.apply_control(control_info, target_velocity, ego_vehicle.get_velocity().length() * 3.6, min_depth)           
            if send_info:
                send_data({"velocity": ego_vehicle.get_velocity().length() * 3.6, "acceleration": ego_vehicle.get_acceleration().length()})
            if save_info:
                logger.write(str(ego_vehicle.get_velocity().length() * 3.6)+ "," + str(ego_vehicle.get_acceleration().length())+ "," +str(control_info.ego_control.throttle)+ "," +str(control_info.ego_control.brake))
            lastUpdate = time.time()

        clock.tick_busy_loop(120)
        if controller.parse_events(world, clock):
            break
        
        world.apply_control(control_info.ego_control)
        other_vehicle.apply_control(control_info.target_control)
        world.render(display, control_info)
        pygame.display.flip()
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    carla_utility.destroy_all_vehicle_and_sensors(client.get_world()) 
    logger.close() 
    if send_info: 
        close_servers()
    if show_log:
        plot_utility.plot_last_run()    

