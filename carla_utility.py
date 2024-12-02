import carla, time, debug_utility

def __spawn_actor(world, blueprint, spawn_point, attach_to = None):
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    time.sleep(2)
    return actor

def __transform_vector(point: carla.Transform, transform: carla.Transform):
    point.location.x += transform.location.x
    point.location.y += transform.location.y
    point.location.z += transform.location.z
    point.rotation.pitch += transform.rotation.pitch
    point.rotation.roll += transform.rotation.roll
    point.rotation.yaw += transform.rotation.yaw
    return point

def move_spectator_to(spectator, to, distance=5.0, transform = carla.Transform(carla.Location(x = 5, z=5), carla.Rotation(pitch=-10))):
    spectator_transform = carla.Transform(carla.Location(to.location - to.get_forward_vector() * distance), to.rotation)
    spectator.set_transform(__transform_vector(spectator_transform, transform))
    
def spawn_vehicle_bp_at(world, vehicle, spawn_point=carla.Transform(), transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(vehicle) 
    return __spawn_actor(world, vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_vehicle_at(world, vehicle_index=0, spawn_point=carla.Transform(), pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]   
    return __spawn_actor(world, vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_vehicle_in_front_of(world, vehicle: carla.Actor, vehicle_index=0, offset=0):
    v3d = debug_utility.get_point_at(vehicle, vehicle.get_location(), offset)
    #+10 on z is to avoid spawn bug
    return spawn_vehicle_at(world,vehicle_index,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 10), vehicle.get_transform().rotation))

def spawn_vehicle_bp_in_front_of(world, vehicle, vehicle_bp_name, offset=0):
    v3d = debug_utility.get_point_at(vehicle, vehicle.get_location(), offset)
    #+10 on z is to avoid spawn bug
    return spawn_vehicle_bp_at(world,vehicle_bp_name,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 10), vehicle.get_transform().rotation))

def spawn_vehicle(world, vehicle_index=0, spawn_index=0, pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    return __spawn_actor(world, vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_camera(world, attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    return __spawn_actor(world, camera_bp, transform, attach_to=attach_to)

def spawn_radar(world, attach_to, transform=carla.Transform(carla.Location(x=1.2, z=2.2), carla.Rotation(pitch=0)), horizontal_fov = 30, vertical_fov = 30, range=100, points_per_second=1500, sensor_tick = 0):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', str(horizontal_fov))
    radar_bp.set_attribute('vertical_fov', str(vertical_fov))
    radar_bp.set_attribute('range', str(range))
    radar_bp.set_attribute('points_per_second', str(points_per_second))
    radar_bp.set_attribute('sensor_tick', str(sensor_tick))
    return __spawn_actor(world, radar_bp, transform, attach_to=attach_to)

def destroy_all_vehicle_and_sensors(world: carla.World):
    for v in world.get_actors().filter('vehicle.*'):
        v.destroy()
    for v in world.get_actors().filter('sensor.*'):
        v.destroy()