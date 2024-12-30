import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility, server, plot_utility
from log_utility import Logger 
from vehicle_controller.pid_controller import pid_controller_random_adaptive, pid_controller_scheduled_adaptive, pid_controller_random, pid_controller_scheduled
from vehicle_controller.rl_controller import rl_controller
from control_utility import ControlInfo, DualControl
from carla_utility import CameraManager

send_info = False
save_info = False
show_log = False
show_in_carla = False
cc = False
update_frequency = 0.01 #seconds

radar_detection_h_radius = 1
radar_detection_v_radius = 0.8
max_target_velocity = 130
radar_range_offset = 20
radar_range = carla_utility.compute_security_distance(max_target_velocity) + radar_range_offset
target_velocity = 90
min_distance_offset = 7
learning_rate = 0.003
spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))

def radar_callback(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_depth, relative_velocity
    min_ttc = min_depth = float('inf')
    distances = []
    velocities = []
    ttc = []

    for detection in data:
        if debug_utility.evaluate_point(radar, detection, radar_detection_h_radius, radar_detection_v_radius):
            #debug_utility.draw_radar_point(self.radar_sensor, detection)
            distances.append(detection.depth)
            velocities.append(detection.velocity)
            ttc.append(detection.depth / abs(detection.velocity) if abs(detection.velocity) != 0 else float('inf'))
    min_ttc = min(ttc) if len(ttc) > 0 else float('inf')
    # min distance corresponding to distance at min_ttc index
    min_depth = distances[ttc.index(min_ttc)] if len(distances) > 0 else float('inf')
    relative_velocity = velocities[ttc.index(min_ttc)] if len(velocities) > 0 else 0
    
def restart():
    global ego_vehicle, other_vehicle, camera_manager, control_info
    carla_utility.destroy_all_vehicle_and_sensors()
    control_info = ControlInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
    ego_vehicle = carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=spawn_point)
    camera_manager = CameraManager(ego_vehicle)
    radar = carla_utility.spawn_radar(ego_vehicle, range=radar_range)
    radar.listen(lambda data: radar_callback(data, radar))
    carla_utility.move_spectator_to(ego_vehicle.get_transform())
    other_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=100)

send_info and server.start_servers()
pygame.init()
pygame.font.init()
display = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
clock = pygame.time.Clock()
controller = DualControl(restart_callback=restart)
logger = Logger()
lastUpdate = 0

# ego_controller = pid_controller_random_adaptive.PIDController(learning_rate) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_adaptive.PIDController(learning_rate, update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_random.PIDController() #add buffer_size = None to disable buffer
ego_controller = pid_controller_scheduled.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = rl_controller.RLController()

restart()
try:
    while control_info.running:
        # debug_utility.draw_radar_bounding_range(radar)
        # debug_utility.draw_radar_point_cloud_range(radar, radar_detection_h_radius, radar_detection_v_radius)
        control_info.reset_ego_control() #comment to make keyboard unrensponsive
        control_info.reset_other_vehicle_control()
        control_info.obstacle_relative_velocity = relative_velocity * 3.6
        if control_info.cc():
            ego_controller.apply_control(control_info, ego_vehicle.get_velocity().length() * 3.6, min_depth)      
        if(time.time() - lastUpdate > update_frequency):  
            if send_info:
                server.send_data({"velocity": ego_vehicle.get_velocity().length() * 3.6, "acceleration": ego_vehicle.get_acceleration().length()})
            if save_info:
                logger.write(str(ego_vehicle.get_velocity().length() * 3.6)+ "," + str(ego_vehicle.get_acceleration().length())+ "," +str(control_info.ego_control.throttle)+ "," +str(control_info.ego_control.brake))
            lastUpdate = time.time()

        clock.tick_busy_loop(120)
        if controller.parse_events(camera_manager, control_info, clock):
            break

        ego_vehicle.apply_control(control_info.ego_control)
        other_vehicle.apply_control(control_info.other_vehicle_control)
        camera_manager.render(display, control_info, ego_vehicle)
        pygame.display.flip()
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    carla_utility.destroy_all_vehicle_and_sensors() 
    logger.close() 
    send_info and server.close_servers()
    show_log and plot_utility.plot_last_run()    