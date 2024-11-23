import carla, time

def __spawn_actor(world, blueprint, spawn_point, attach_to = None):
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    time.sleep(1)
    return actor

def move_spectator_to(spectator, to, distance=5.0, transform = carla.Transform(carla.Location(z=4), carla.Rotation(pitch=-30))):
    back_location = to.location - to.get_forward_vector() * distance
    
    back_location.x += transform.location.x
    back_location.y += transform.location.y
    back_location.z += transform.location.z
    to.rotation.yaw += transform.rotation.yaw
    to.rotation.pitch = transform.rotation.pitch
    to.rotation.roll = transform.rotation.roll
    
    spectator_transform = carla.Transform(back_location, to.rotation)
    spectator.set_transform(spectator_transform)

def spawn_veichle_at(world, vehicle_index=0, spawn_point=carla.Transform(), pattern='vehicle.*', transform = carla.Transform()):
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

def draw_on_screen(world, transform, content='O', color=carla.Color(0, 255, 0), life_time=20):
    world.debug.draw_string(transform.location, content, color=color, life_time=life_time)   