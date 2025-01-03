import carla, time, debug_utility, math_utility, re, pygame, numpy as np, weakref, collections, math
from carla import ColorConverter as cc
from vehicle_controller.rl_controller import rl_controller
client = carla.Client('localhost', 2000)
client.set_timeout(100.0)
world = client.get_world()
spectator = world.get_spectator()

def __find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]
        
weather_presets = __find_weather_presets()
weather_index = 0

def next_weather(reverse=False):
    global weather_index
    weather_index += -1 if reverse else 1
    weather_index %= len(weather_presets)
    preset = weather_presets[weather_index]
    world.set_weather(preset[0])

def compute_security_distance(velocity):
    return (velocity // 10) ** 2

radar_detection_h_radius = 1
radar_detection_v_radius = 1
radar_range_offset = 20
max_target_velocity = 130
radar_range = compute_security_distance(max_target_velocity) + radar_range_offset

def move_spectator_to(to, distance=10.0, transform = carla.Transform(carla.Location(z=5), carla.Rotation(pitch=-10))):
    spectator_transform = carla.Transform(carla.Location(to.location - to.get_forward_vector() * distance), to.rotation)
    spectator.set_transform(__transform_vector(spectator_transform, transform))
    
def setup_spectator(spawn_point: carla.Transform):
    #try go back to 1000
    if math_utility.sub(spectator.get_location(), spawn_point.location).length() > 2000:
        move_spectator_to(spawn_point)
        time.sleep(10)

def __spawn_actor(blueprint, spawn_point: carla.Transform, attach_to: carla.Actor = None):
    setup_spectator(carla.Transform(math_utility.add(spawn_point.location, attach_to.get_location() if attach_to != None else carla.Location())))
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    if isinstance(actor, carla.Vehicle):
        actor.apply_control(carla.VehicleControl(brake=0.5))
    time.sleep(2)
    return actor

old_spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
mid_lane_wp = world.get_map().get_waypoint(carla.Location(x=2390, y=6110, z=187), True)
left_lane_wp = world.get_map().get_waypoint(carla.Location(x=2385, y=6110, z=187), True)

def spawn(way_point: carla.Waypoint, spawn_distance):
    blueprint_library = world.get_blueprint_library().filter('vehicle.*')[0]
    # vehicle_bp = np.random.choice(blueprint_library)
    spawn_point = way_point.next(spawn_distance)[0].transform
    spawn_point.location.z += 2
    return __spawn_actor(blueprint_library, spawn_point)

def get_wp_from_lane_id(lane_id):
    if lane_id == mid_lane_wp.lane_id:
        return mid_lane_wp
    elif lane_id == mid_lane_wp.get_left_lane().lane_id:
        return mid_lane_wp.get_left_lane()
    else:
        return mid_lane_wp.get_right_lane()

def handle_traffic_same_spawn(vehicles):
    for v in vehicles:
        current_wp = world.get_map().get_waypoint(v[0].get_location(), True)
        current_wp_spawn = get_wp_from_lane_id(current_wp.lane_id)
        if current_wp_spawn.transform.location.distance(v[0].get_location()) > 100:
            spawn_point = get_wp_from_lane_id(v[1]).next(30)[0].transform
            spawn_point.location.z += 2
            v[0].set_transform(spawn_point)

def spawn_traffic_same_spawn(vehicle_per_lane: int):
    spawn_distance = 30
    vehicles = []
    for i in range(vehicle_per_lane):
            vehicles.append([spawn(mid_lane_wp, spawn_distance * (i + 1)), mid_lane_wp.lane_id])
            vehicles.append([spawn(mid_lane_wp.get_right_lane(), spawn_distance * (i + 1)), mid_lane_wp.get_right_lane().lane_id])
    for v in vehicles:
        v[0].set_autopilot(True)
    return lambda: handle_traffic_same_spawn(vehicles)

def handle_traffic(vehicles):
    for v in vehicles:
        current_wp = world.get_map().get_waypoint(v.get_location(), True)
        current_wp_spawn = get_wp_from_lane_id(current_wp.lane_id)
        if current_wp_spawn.transform.location.distance(v.get_location()) > 100:
            spawn_point = current_wp_spawn.next(30)[0].transform
            spawn_point.location.z += 2
            v.set_transform(spawn_point)

def spawn_traffic(num_vehicle: int):
    spawn_distance = 10
    vehicles = []
    for i in range(num_vehicle):
            vehicles.append(spawn(mid_lane_wp, spawn_distance * (i + 1)))
    for v in vehicles:
        v.set_autopilot(True)
    return lambda: handle_traffic(vehicles)

def __transform_vector(point: carla.Transform, transform: carla.Transform):
    point.location.x += transform.location.x
    point.location.y += transform.location.y
    point.location.z += transform.location.z
    point.rotation.pitch += transform.rotation.pitch
    point.rotation.roll += transform.rotation.roll
    point.rotation.yaw += transform.rotation.yaw
    return point
    
def spawn_vehicle_bp_at(vehicle, spawn_point=carla.Transform(), transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(vehicle) 
    return __spawn_actor(vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_vehicle_at(vehicle_index=0, spawn_point=carla.Transform(), pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]   
    return __spawn_actor(vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_vehicle_in_front_of(vehicle: carla.Actor, vehicle_index=0, offset=0):
    v3d = debug_utility.get_point_at(vehicle, vehicle.get_location(), offset)
    #+10 on z is to avoid spawn bug
    return spawn_vehicle_at(vehicle_index,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 10), vehicle.get_transform().rotation))

def spawn_vehicle_bp_in_front_of(vehicle, vehicle_bp_name, offset=0):
    v3d = debug_utility.get_point_at(vehicle, vehicle.get_location(), offset)
    #+10 on z is to avoid spawn bug @TODO
    return spawn_vehicle_bp_at(vehicle_bp_name,spawn_point=carla.Transform(carla.Location(v3d.x, v3d.y, v3d.z + 2), vehicle.get_transform().rotation))

def spawn_vehicle(vehicle_index=0, spawn_index=0, pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    return __spawn_actor(vehicle_bp, __transform_vector(spawn_point, transform))

def spawn_camera(attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    return __spawn_actor(camera_bp, transform, attach_to=attach_to)

def spawn_radar(attach_to, transform=carla.Transform(carla.Location(x=1.2, z=2.2), carla.Rotation(pitch=0)), horizontal_fov = 30, vertical_fov = 30, range=100, points_per_second=1500, sensor_tick = 0):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', str(horizontal_fov))
    radar_bp.set_attribute('vertical_fov', str(vertical_fov))
    radar_bp.set_attribute('range', str(range))
    radar_bp.set_attribute('points_per_second', str(points_per_second))
    radar_bp.set_attribute('sensor_tick', str(sensor_tick))
    return __spawn_actor(radar_bp, transform, attach_to=attach_to)

def destroy_all_vehicle_and_sensors():
    for v in world.get_actors().filter('vehicle.*'):
        v.destroy()
    for v in world.get_actors().filter('sensor.*'):
        v.stop()
        v.destroy()
    time.sleep(2)    

class VehicleWithRadar:
    def __init__(self, vehicle, acc_info, veichle_controller = rl_controller.RLController(), show_detection=False, show_range=False, show_filter=False):
        self.vehicle = vehicle
        self.acc_info = acc_info
        self.radar_detection_h_radius = radar_detection_h_radius
        self.radar_detection_v_radius = radar_detection_v_radius
        self.vehicle_controller = veichle_controller
        self.show_detection = show_detection
        self.show_range = show_range
        self.show_filter = show_filter
        self.radar = spawn_radar(vehicle, range=radar_range)
        self.radar.listen(self.__radar_callback)
        time.sleep(1)

    def __radar_callback(self, data):
        self.min_ttc = self.min_depth = self.relative_velocity = float('inf')
        distances = []
        velocities = []
        ttc = []

        for detection in data:
            if debug_utility.evaluate_point(self.radar, detection, self.radar_detection_h_radius, self.radar_detection_v_radius):
                self.show_detection and debug_utility.draw_radar_point(self.radar_sensor, detection)
                distances.append(detection.depth)
                velocities.append(detection.velocity)
                ttc.append(detection.depth / abs(detection.velocity) if abs(detection.velocity) != 0 else float('inf'))
        self.min_ttc = min(ttc) if len(ttc) > 0 else float('inf')
        self.min_depth = distances[ttc.index(self.min_ttc)] if len(distances) > 0 else float('inf')
        self.relative_velocity = velocities[ttc.index(self.min_ttc)] if len(velocities) > 0 else float('inf')

    def compute_control(self):
        if self.acc_info.is_active():
            self.vehicle_control = self.vehicle_controller.apply_control(self.acc_info, self.vehicle.get_velocity().length() * 3.6, self.min_depth)
        else:
            self.vehicle_control = carla.VehicleControl()
        return self.vehicle_control
    
    def apply_control(self):
        self.vehicle.apply_control(self.vehicle_control)

    def compute_and_apply_control(self):
        self.compute_control()
        self.apply_control() 

    def get_location(self):
        return self.vehicle.get_location()
    
    def get_velocity(self):
        return self.vehicle.get_velocity()
    
    def get_transform(self):
        return self.vehicle.get_transform()
    
def print_text_to_screen(display, text, position, color):
        font = pygame.font.Font(None, 36)  # You can choose the font and size
        text_surface = font.render(text, True, color)  # White color
        display.blit(text_surface, position)

class CameraManager(object):
    def __init__(self, parent_actor, transform_index = 0):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = transform_index
        # self.hud = hud
        self.sensors = ['sensor.camera.rgb', cc.Raw, 'Camera RGB']
        self.sensors.append(self._parent.get_world().get_blueprint_library().find(self.sensors[0]))
        self.sensors[-1].set_attribute('image_size_x', "1280")
        self.sensors[-1].set_attribute('image_size_y', "720")
        # if item[0].startswith('sensor.camera'):
        #     bp.set_attribute('image_size_x', str(hud.dim[0]))
        #     bp.set_attribute('image_size_y', str(hud.dim[1]))

        self.__spawn_camera()    

    def __spawn_camera(self):
        self.sensor = self._parent.get_world().spawn_actor(
            self.sensors[-1],
            self._camera_transforms[self.transform_index],
            attach_to=self._parent)
    
        self.sensor.listen(self._parse_image)

    #change camera view
    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.destroy()
        self.__spawn_camera()


    def render(self, display, control_info, ego_vehicle: carla.Vehicle):
        if self.surface is not None:
            scaled_surface = pygame.transform.scale(self.surface, display.get_size())
            display.blit(scaled_surface, (0, 0))
        print_text_to_screen(display, f"Throttle: {control_info.ego_control.throttle}", (10, 10), (255, 255, 255))
        print_text_to_screen(display, f"Brake: {control_info.ego_control.brake}", (10, 50), (255, 255, 255))
        print_text_to_screen(display, f"Steer: {control_info.ego_control.steer}", (10, 90), (255, 255, 255))
        print_text_to_screen(display, f"Hand Brake: {control_info.ego_control.hand_brake}", (10, 130), (255, 255, 255))
        print_text_to_screen(display, f"PID CC: {control_info.acc_info.is_active()}", (10, 170), (255, 255, 255))
        print_text_to_screen(display, f"Min Permitted Distance: {control_info.acc_info.min_permitted_offset}", (10, 210), (255, 255, 255))
        print_text_to_screen(display, f"Ego Velocity: {ego_vehicle.get_velocity().length() * 3.6}", (10, 250), (255, 255, 255))
        print_text_to_screen(display, f"Target Velocity: {control_info.acc_info.target_velocity}", (10, 290), (255, 255, 255))
        other_vehicle_velocity = ego_vehicle.get_velocity().length() * 3.6 + control_info.obstacle_relative_velocity
        print_text_to_screen(display, f"Obstacle Velocity: {other_vehicle_velocity}", (10, 330), (255, 255, 255))

    def _parse_image(self, image):
        image.convert(self.sensors[1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

class CollisionSensor(object):
    def __init__(self, parent_actor, display):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.display = display
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event, self.display))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event, display):
        self = weak_self()
        if not self:
            return
        # Display the collision on the screen
        print_text_to_screen(display, f"Collision at {event.frame}", (10, 370), (255, 255, 255))
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)