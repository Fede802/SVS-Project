import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, time, pygame, cv2, debug_utility, carla_utility, server, plot_utility
from log_utility import Logger 
from vehicle_controller.pid_controller import pid_controller_random_adaptive, pid_controller_scheduled_adaptive, pid_controller_random, pid_controller_scheduled
from vehicle_controller.rl_controller import rl_controller
from control_utility import ControlInfo, DualControl, ACCInfo
from carla_utility import CameraManager, VehicleWithRadar, CollisionSensor

send_info = False
save_info = False
show_log = False
show_in_carla = False
cc = False
update_frequency = 0.01 #seconds




target_velocity = 90
min_distance_offset = 7
learning_rate = 0.003
spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
spawn_lane3 = carla.Transform(carla.Location(x=2385, y=6110, z=187), carla.Rotation(yaw = -85.54))
spawn_lane3 = carla_utility.world.get_map().get_waypoint(spawn_lane3.location, True).transform
spawn_lane3.location.z = 189
spawn_lane2 = carla.Transform(carla.Location(x=2390, y=6110, z=187), carla.Rotation(yaw = -85.54))
spawn_lane2 = carla_utility.world.get_map().get_waypoint(spawn_lane2.location, True).transform
spawn_lane2.location.z = 189
    
def restart():
    global ego_vehicle, other_vehicle, camera_manager, control_info, display, collision_sensor, cb
    carla_utility.destroy_all_vehicle_and_sensors()
    acc_info = ACCInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=target_velocity)
    control_info = ControlInfo(acc_info)
    ego_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=spawn_lane3), ego_controller)
    ego_vehicle.acc_info = acc_info
    camera_manager = CameraManager(ego_vehicle.vehicle)
    #collision_sensor = CollisionSensor(ego_vehicle.vehicle, display)
    carla_utility.move_spectator_to(ego_vehicle.vehicle.get_transform())
    other_vehicle = VehicleWithRadar(carla_utility.spawn_vehicle_bp_at(vehicle='vehicle.tesla.cybertruck', spawn_point=spawn_lane2))
    other_vehicle.acc_info = ACCInfo(cc, min_permitted_offset=min_distance_offset, target_velocity=0)
    # other_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=100)
    cb = carla_utility.spawn_traffic(2, [spawn_lane3, spawn_lane2])

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
# carla_utility.move_spectator_to(carla.Transform(carla.Location(x=2385, y=6119, z=169.070175), carla.Rotation(pitch=5.283404, yaw=-85.546600, roll=0.026908)))
# carla_utility.move_spectator_to(carla.Transform(carla.Location(x=2390, y=6120, z=169.070175), carla.Rotation(pitch=5.283404, yaw=-85.546600, roll=0.026908)))
 
try:
    while control_info.running:
        cb()
        control_info.obstacle_relative_velocity = ego_vehicle.relative_velocity * 3.6
        control_info.ego_control = ego_vehicle.compute_control()
        control_info.other_vehicle_control = other_vehicle.compute_control()

        # if control_info.cc():
        #     ego_controller.apply_control(control_info, ego_vehicle.get_velocity().length() * 3.6, min_depth)      
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
        other_vehicle.apply_control()
        camera_manager.render(display, control_info, ego_vehicle.vehicle)
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