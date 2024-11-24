import carla, time, debug_utility

def __spawn_actor(world, blueprint, spawn_point, attach_to = None):
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    time.sleep(1)
    return actor

def move_spectator_to(world, spectator, to, distance=5.0, transform = carla.Transform(carla.Location(x = 5, z=5), carla.Rotation(pitch=-30))):
    back_location = to.location - to.get_forward_vector() * distance
    back_location.x += transform.location.x
    back_location.y += transform.location.y
    back_location.z += transform.location.z
    to.rotation.yaw += transform.rotation.yaw
    to.rotation.pitch = transform.rotation.pitch
    to.rotation.roll = transform.rotation.roll
    spectator_transform = carla.Transform(back_location, to.rotation)
    spectator.set_transform(spectator_transform)
    #world.tick()

def spawn_vehicle_bp_at(world, vehicle, spawn_point=carla.Transform(), transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(vehicle)   
    spawn_point.location.x += transform.location.x
    spawn_point.location.y += transform.location.y
    spawn_point.location.z += transform.location.z
    spawn_point.rotation.pitch += transform.rotation.pitch
    spawn_point.rotation.roll += transform.rotation.roll
    spawn_point.rotation.yaw += transform.rotation.yaw
    return __spawn_actor(world, vehicle_bp, spawn_point)

def spawn_vehicle_at(world, vehicle_index=0, spawn_point=carla.Transform(), pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]   
    spawn_point.location.x += transform.location.x
    spawn_point.location.y += transform.location.y
    spawn_point.location.z += transform.location.z
    spawn_point.rotation.pitch += transform.rotation.pitch
    spawn_point.rotation.roll += transform.rotation.roll
    spawn_point.rotation.yaw += transform.rotation.yaw
    return __spawn_actor(world, vehicle_bp, spawn_point)

def spawn_vehicle(world, vehicle_index=0, spawn_index=0, pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    spawn_point.location.x += transform.location.x
    spawn_point.location.y += transform.location.y
    spawn_point.location.z += transform.location.z
    spawn_point.rotation.pitch += transform.rotation.pitch
    spawn_point.rotation.roll += transform.rotation.roll
    spawn_point.rotation.yaw += transform.rotation.yaw
    return __spawn_actor(world, vehicle_bp, spawn_point)

def spawn_camera(world, attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    #camera_bp.set_attribute('fov', '120')
    #camera_bp.set_attribute('sensor_tick', '0')
    return __spawn_actor(world, camera_bp, transform, attach_to=attach_to)

def spawn_radar(world, attach_to, transform=carla.Transform(carla.Location(x=1.2, z=1.2)), range=20, points_per_second=10):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '20')
    radar_bp.set_attribute('vertical_fov', '20')
    radar_bp.set_attribute('range', str(range))
    #radar_bp.set_attribute('points_per_second', str(points_per_second))
    return __spawn_actor(world, radar_bp, transform, attach_to=attach_to)

def draw_on_screen(world, transform, content='O', color=carla.Color(0, 255, 0), life_time=0.1):
    world.debug.draw_string(transform.location, content, color=color, life_time=life_time)   


def compute_cruise_controls(velocita_corrente, velocita_target, ttc, k_p=0.05):
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

    # If 
    if ttc < 1.5:
        brake = 1.0
        throttle = 0.0
    elif ttc < 3.0:
        brake = 0.5 
        throttle = 0.0
    else:
    # Velocity Check, without obstacles
        if errore_vel > 0:
            throttle = 1.0 * errore_vel_perc
        else:
            brake = 1.0 * errore_vel_perc    
       
    # if errore_vel < 0:
    # #    Decelerazione
    #     brake = min(-k_p * errore_vel, 1.0)

    #if brake > 0:
     #   throttle = 0.0
    
    # TTC Safety Override
    #if ttc < 1.5:
     #   brake = 1.0
      #  throttle = 0.0
    
    return throttle, brake