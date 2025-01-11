import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility, server, plot_utility, traffic_example, threading
from log_utility import Logger 
from vehicle_controller.pid_controller import pid_controller_random_adaptive, pid_controller_scheduled_adaptive, pid_controller_random, pid_controller_scheduled, pid_controller_scheduled_following, pid_controller_scheduled_following_brake, pid_controller_scheduled_following_brake_ttc, pid_controller_random_following_brake_ttc
from vehicle_controller.rl_controller import rl_controller
from control_utility import ProgramInfo, DualControl, ACCInfo
from carla_utility import CameraManager, VehicleWithRadar, CollisionSensor, FadingText

send_info = True
save_info = True
show_log = True
show_in_carla = False
cc = True
update_frequency = 0.01 #seconds

target_velocity = 110
min_distance_offset = 7
learning_rate = 0.003

traffic_thread = None
stop_event = threading.Event()

map_list = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07', 'Town10HD', 'Town11', 'Town12', 'Town15']
current_map = 0
# mode 1 = town13 two car
# mode 2 = town13 traffic
# mode 3 = town10 traffic
def restart(mode = 1):
    global ego_vehicle, other_vehicle, camera_manager, program_info, display, collision_sensor, fading_text, cb, traffic_thread, stop_event, current_map
    
    if traffic_thread != None:
        stop_event.set()
        carla_utility.world.tick()
        traffic_thread.join()
        stop_event.clear()
    other_vehicle = cb = traffic_thread = None
    carla_utility.destroy_all_vehicle_and_sensors()
    ego_acc_info = ACCInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
    program_info = ProgramInfo(ego_acc_info, send_info=send_info)
    if mode == 3 or mode == 4:
        if mode == 4:
            current_map = (current_map + 1) % len(map_list)
        carla_utility.load_world(map_list[current_map]) 
        ego_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp(vehicle='vehicle.tesla.cybertruck'), ego_acc_info, ego_controller)
        traffic_thread = threading.Thread(target=traffic_example.main, args=(stop_event,), daemon=True)
        traffic_thread.start()
    else:
        carla_utility.load_world('Town13')
        ego_spawn_point = carla_utility.mid_lane_wp.transform
        ego_spawn_point.location.z += 2
        ego_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=ego_spawn_point), ego_acc_info, ego_controller)
        if mode == 2:
            cb = carla_utility.spawn_traffic(8)
        else:
            other_vehicle_spawn_point = carla_utility.mid_lane_wp.next(500)[0].transform
            other_vehicle_spawn_point.location.z += 2
            other_vehicle_acc_info = ACCInfo(True, min_permitted_offset=min_distance_offset, target_velocity=110)
            other_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=other_vehicle_spawn_point), other_vehicle_acc_info)
    camera_manager = CameraManager(display, ego_vehicle.vehicle)
    # little text in the center of the screen
    fading_text = FadingText(pygame.font.Font(pygame.font.get_default_font(), 24))
    collision_sensor = CollisionSensor(ego_vehicle.vehicle, fading_text, program_info)
    carla_utility.move_spectator_to(ego_vehicle.vehicle.get_transform())

if send_info:
    server.start_servers()
    server.send_data("Program Start")

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
# ego_controller = pid_controller_scheduled_following.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_following_brake.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_random_following_brake_ttc.PIDController() #add buffer_size = None to disable buffer
ego_controller = pid_controller_scheduled_following_brake_ttc.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = rl_controller.RLController()
restart()

try:
    while program_info.running:
        cb != None and cb()
        program_info.obstacle_relative_velocity = ego_vehicle.relative_velocity * 3.6
        program_info.ego_control = ego_vehicle.compute_control()
        program_info.other_vehicle_control = other_vehicle.compute_control() if other_vehicle != None else carla.VehicleControl()
        program_info.ego_velocity = ego_vehicle.vehicle.get_velocity().length() * 3.6
        
        if(time.time() - lastUpdate > update_frequency):  
            # if send_info:
            #     server.send_data({"velocity": ego_vehicle.vehicle.get_velocity().length() * 3.6, "acceleration": ego_vehicle.vehicle.get_acceleration().length()})
            if save_info:
                logger.write(str(ego_vehicle.vehicle.get_velocity().length() * 3.6)+ "," + str(ego_vehicle.vehicle.get_acceleration().length())+ "," +str(program_info.ego_control.throttle)+ "," +str(program_info.ego_control.brake))
                # Log brake and distance to obstacle
            lastUpdate = time.time()

        clock.tick_busy_loop(120)
        if controller.parse_events(camera_manager, program_info, clock):
            break

        ego_vehicle.apply_control()
        other_vehicle != None and other_vehicle.apply_control()
        camera_manager.render(display, program_info)
        fading_text.tick(ego_vehicle.vehicle.get_world(), clock=clock)
        fading_text.render(display)
        carla_utility.setup_spectator(ego_vehicle.vehicle.get_transform())
        carla_utility.world.tick()
        pygame.display.flip()
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    carla_utility.destroy_all_vehicle_and_sensors() 
    logger.close() 
    if send_info:
        server.send_data("Program End")
        server.close_servers()
    show_log and plot_utility.plot_last_run()    