import carla, time, debug_utility

def __spawn_actor(world, blueprint, spawn_point, attach_to = None):
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    print(actor.attributes)
    time.sleep(10)
    return actor

def move_spectator_to(world, to, distance=5.0, transform = carla.Transform(carla.Location(x = 5, z=5), carla.Rotation(pitch=-10))):
    back_location = to.location - to.get_forward_vector() * distance
    back_location.x += transform.location.x
    back_location.y += transform.location.y
    back_location.z += transform.location.z
    to.rotation.yaw += transform.rotation.yaw
    to.rotation.pitch = transform.rotation.pitch
    to.rotation.roll = transform.rotation.roll
    spectator_transform = carla.Transform(back_location, to.rotation)
    world.get_spectator().set_transform(spectator_transform)
    #world.tick()

def spawn_veichle_bp_at(world, vehicle, spawn_point=carla.Transform(), transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(vehicle)   
    spawn_point.location.x += transform.location.x
    spawn_point.location.y += transform.location.y
    spawn_point.location.z += transform.location.z
    spawn_point.rotation.pitch += transform.rotation.pitch
    spawn_point.rotation.roll += transform.rotation.roll
    spawn_point.rotation.yaw += transform.rotation.yaw
    return __spawn_actor(world, vehicle_bp, spawn_point)

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

def spawn_veichle_in_front_of(world, veichle, vehicle_index=0, offset=0):
    v3d = debug_utility.get_point_at(veichle, offset)
    return spawn_veichle_at(world,vehicle_index,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 10), veichle.get_transform().rotation))

def spawn_veichle_bp_in_front_of(world, veichle, vehicle_bp_name, offset=0):
    v3d = debug_utility.get_point_at(veichle, offset)
    return spawn_veichle_bp_at(world,vehicle_bp_name,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 30), veichle.get_transform().rotation))

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

def spawn_radar(world, attach_to, transform=carla.Transform(carla.Location(x=1.2, z=2), carla.Rotation(pitch=0)), range=10, points_per_second=10000):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '1')
    radar_bp.set_attribute('vertical_fov', '1')
    radar_bp.set_attribute('range', str(range))
    radar_bp.set_attribute('points_per_second', str(points_per_second))
    return __spawn_actor(world, radar_bp, transform, attach_to=attach_to)

def spawn_lidar(world, attach_to, transform=carla.Transform(carla.Location(x=1.2, z=2), carla.Rotation(pitch=0))):
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', str(100))
    lidar_bp.set_attribute('rotation_frequency', str(0.1))
    lidar_bp.set_attribute('dropoff_general_rate', str(0))
    lidar_bp.set_attribute('dropoff_intensity_limit', str(0))
    lidar_bp.set_attribute('dropoff_zero_intensity', str(0))

    return __spawn_actor(world, lidar_bp, transform, attach_to=attach_to)

def draw_on_screen(world, transform, content='O', color=carla.Color(0, 255, 0), life_time=0.1):
    world.debug.draw_string(transform.location, content, color=color, life_time=life_time)   
