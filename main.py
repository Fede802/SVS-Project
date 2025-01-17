import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility, server, plot_utility, traffic_example, threading
from log_utility import Logger 
from vehicle_controller.pid_controller import pid_controller_random_adaptive, pid_controller_scheduled_adaptive, pid_controller_random, pid_controller_scheduled, pid_controller_scheduled_following, pid_controller_scheduled_following_brake, pid_controller_scheduled_following_brake_ttc, pid_controller_random_following_brake_ttc
from vehicle_controller.rl_controller import rl_controller
from control_utility import ProgramInfo, DualControl, ACCInfo
from carla_utility import CameraManager, VehicleWithRadar, CollisionSensor, FadingText

learning_rate = 0.003

traffic_thread = None
stop_event = threading.Event()

map_list = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07', 'Town10HD', 'Town11', 'Town12', 'Town15']
current_map = 7
# mode 1 = town13 two car
# mode 2 = town13 traffic
# mode 3 = other_town_reset traffic
# mode 4 = other_town switch
# mode 5 = single car
# mode 6 = demo
def restart(mode = 1):
    global ego_vehicle, other_vehicle, camera_manager, program_info, display, collision_sensor, fading_text, cb, traffic_thread, stop_event, current_map, demo
    demo = False
    send_info and server.send_data("Program Restart")
    if traffic_thread != None:
        stop_event.set()
        carla_utility.world.tick()
        traffic_thread.join()
        stop_event.clear()
    other_vehicle = cb = traffic_thread = None
    carla_utility.destroy_all_vehicle_and_sensors()
    ego_acc_info = ACCInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
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
        elif mode == 1:
            other_vehicle_spawn_point = carla_utility.mid_lane_wp.next(130)[0].transform
            other_vehicle_spawn_point.location.z += 2
            other_vehicle_acc_info = ACCInfo(True, min_permitted_offset=min_distance_offset, target_velocity=70)
            other_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=other_vehicle_spawn_point), other_vehicle_acc_info)
        elif mode == 6:
            demo = True
            other_vehicle_spawn_point = carla_utility.mid_lane_wp.next(130)[0].transform
            other_vehicle_spawn_point.location.z += 2
            other_vehicle_acc_info = ACCInfo(True, min_permitted_offset=min_distance_offset, target_velocity=70)
            other_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=other_vehicle_spawn_point), other_vehicle_acc_info)
                
    camera_manager = CameraManager(display, ego_vehicle.vehicle)
    fading_text = FadingText(pygame.font.Font(pygame.font.get_default_font(), 24))
    program_info = ProgramInfo(ego_vehicle, other_vehicle, send_info=send_info)
    collision_sensor = CollisionSensor(ego_vehicle.vehicle, fading_text, program_info)
    carla_utility.move_spectator_to(ego_vehicle.vehicle.get_transform())

send_info = True
save_info = True
show_log = True
cc = True
update_frequency = 0.01 #seconds

target_velocity = 90
min_distance_offsets = [7, 14, 21]
min_distance_offset = min_distance_offsets[0]

if send_info:
    server.start_servers()
spawn_time = 0
pygame.init()
pygame.font.init()
display = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)

clock = pygame.time.Clock()
controller = DualControl(restart_callback=restart, target_velocity=target_velocity)
logger = Logger()
lastUpdate = 0

# ego_controller = pid_controller_random_adaptive.PIDController(learning_rate) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_adaptive.PIDController(learning_rate, update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_random.PIDController() #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_following.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_following_brake.PIDController(update_frequency) #add buffer_size = None to disable buffer
# ego_controller = pid_controller_random_following_brake_ttc.PIDController() #add buffer_size = None to disable buffer
# ego_controller = pid_controller_scheduled_following_brake_ttc.PIDController(update_frequency) #add buffer_size = None to disable buffer
ego_controller = rl_controller.RLController()
restart(6)
try:
    while program_info.running:
        cb != None and cb()
        program_info.reset()
        if(time.time() - lastUpdate > update_frequency): 
            spawn_time += 1 
            if send_info:
                server.send_data({"velocity": ego_vehicle.vehicle.get_velocity().length() * 3.6, "acceleration": ego_vehicle.vehicle.get_acceleration().length()})
            if save_info:
                logger.write(str(ego_vehicle.vehicle.get_velocity().length() * 3.6)+ "," + str(ego_vehicle.vehicle.get_acceleration().length())+ "," +str(program_info.ego_control.throttle)+ "," +str(program_info.ego_control.brake))
            lastUpdate = time.time()

        clock.tick_busy_loop(120)
        if controller.parse_events(camera_manager, program_info, clock):
            break
        if spawn_time >= 50:
            ego_vehicle.apply_control()
            if demo and 800 < spawn_time < 1000:
                other_vehicle.vehicle_control = carla.VehicleControl(brake=1)
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