import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility, server, plot_utility, traffic_example, threading
from log_utility import Logger 
from vehicle_controller.pid_controller import pid_controller_random_adaptive, pid_controller_scheduled_adaptive, pid_controller_random, pid_controller_scheduled
from vehicle_controller.rl_controller import rl_controller
from control_utility import ControlInfo, DualControl, ACCInfo
from carla_utility import CameraManager, VehicleWithRadar, CollisionSensor, FadingText

send_info = False
save_info = False
show_log = False
show_in_carla = False
cc = False
update_frequency = 0.01 #seconds

target_velocity = 90
min_distance_offset = 7
learning_rate = 0.003

traffic_thread = None
stop_event = threading.Event()
# mode 1 = town13 two car
# mode 2 = town13 traffic
# mode 3 = town10 traffic
def restart(mode = 1):
    global ego_vehicle, other_vehicle, camera_manager, control_info, display, collision_sensor, fading_text, cb, traffic_thread, stop_event
    
    if traffic_thread != None:
        stop_event.set()
        traffic_thread.join()
        stop_event.clear()
    other_vehicle = cb = traffic_thread = None
    carla_utility.destroy_all_vehicle_and_sensors()
    ego_acc_info = ACCInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
    control_info = ControlInfo(ego_acc_info)
    if mode == 3:
        carla_utility.load_world('Town10HD') 
        traffic_thread = threading.Thread(target=traffic_example.main, args=(stop_event,), daemon=True)
        traffic_thread.start()
    else:
        carla_utility.load_world('Town13')
        ego_spawn_point = carla_utility.mid_lane_wp.transform
        ego_spawn_point.location.z += 2
        if mode == 2:
            cb = carla_utility.spawn_traffic(5)
        else:
            other_vehicle_spawn_point = carla_utility.mid_lane_wp.next(100)[0].transform
            other_vehicle_spawn_point.location.z += 2
            other_vehicle_acc_info = ACCInfo(True, min_permitted_offset=min_distance_offset, target_velocity=40)
            other_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=other_vehicle_spawn_point), other_vehicle_acc_info)
    ego_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=ego_spawn_point), ego_acc_info, ego_controller)
    camera_manager = CameraManager(ego_vehicle.vehicle)
    # little text in the center of the screen
    fading_text = FadingText(pygame.font.Font(pygame.font.get_default_font(), 12), (1280, 720), (0, 720 - 40))
    collision_sensor = CollisionSensor(ego_vehicle.vehicle, display, fading_text)
    carla_utility.move_spectator_to(ego_vehicle.vehicle.get_transform())
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
# ego_controller = pid_controller_scheduled.PIDController(update_frequency) #add buffer_size = None to disable buffer
ego_controller = rl_controller.RLController()

restart()
 
try:
    while control_info.running:
        cb != None and cb()
        control_info.obstacle_relative_velocity = ego_vehicle.relative_velocity * 3.6
        control_info.ego_control = ego_vehicle.compute_control()
        control_info.other_vehicle_control = other_vehicle.compute_control() if other_vehicle != None else carla.VehicleControl()
      
        if(time.time() - lastUpdate > update_frequency):  
            if send_info:
                server.send_data({"velocity": ego_vehicle.vehicle.get_velocity().length() * 3.6, "acceleration": ego_vehicle.vehicle.get_acceleration().length()})
            if save_info:
                logger.write(str(ego_vehicle.vehicle.get_velocity().length() * 3.6)+ "," + str(ego_vehicle.vehicle.get_acceleration().length())+ "," +str(control_info.ego_control.throttle)+ "," +str(control_info.ego_control.brake))
            lastUpdate = time.time()

        clock.tick_busy_loop(120)
        if controller.parse_events(camera_manager, control_info, clock):
            break

        ego_vehicle.apply_control()
        other_vehicle != None and other_vehicle.apply_control()
        camera_manager.render(display, control_info, ego_vehicle.vehicle)
        fading_text.tick(ego_vehicle.vehicle.get_world(), clock=clock)
        fading_text.render(display)
        carla_utility.setup_spectator(ego_vehicle.vehicle.get_transform())

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