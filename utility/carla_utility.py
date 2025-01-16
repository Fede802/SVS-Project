import carla, time, debug_utility, math_utility, re, pygame, numpy as np, weakref, collections, math, random as rnd
from carla import ColorConverter as cc
from vehicle_controller.rl_controller import rl_controller

import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import server


client = carla.Client('localhost', 2000)
client.set_timeout(100.0)

def set_synchronous_mode(synchronous_mode=True):
    settings = world.get_settings()
    settings.synchronous_mode = synchronous_mode
    settings.fixed_delta_seconds = 0.05 if synchronous_mode else None
    world.apply_settings(settings)

world = client.get_world()
sync_world = True
sync_world and set_synchronous_mode()
spectator = world.get_spectator()
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)
tm.set_global_distance_to_leading_vehicle(2.5)

def load_world(world_name):
    global world, spectator, mid_lane_wp, left_lane_wp
    if world.get_map().name.split("/")[-1] != world_name:
        print('Loading world', world_name)
        world = client.load_world(world_name)
        sync_world and world.tick() and set_synchronous_mode()
        world.tick()
        spectator = world.get_spectator()
        if world_name == 'Town13':
            mid_lane_wp = world.get_map().get_waypoint(carla.Location(x=2390, y=6110, z=187), True)
            left_lane_wp = world.get_map().get_waypoint(carla.Location(x=2385, y=6110, z=187), True)        

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
    return (velocity / 10) ** 2

radar_detection_h_radius = 1
radar_detection_v_radius = 0.8
radar_range_offset = 20
max_target_velocity = 130
radar_range = compute_security_distance(max_target_velocity) + radar_range_offset

def move_spectator_to(to, distance=10.0, transform = carla.Transform(carla.Location(z=5), carla.Rotation(pitch=-10))):
    spectator_transform = carla.Transform(carla.Location(to.location - to.get_forward_vector() * distance), to.rotation)
    spectator.set_transform(__transform_vector(spectator_transform, transform))

def setup_spectator(spawn_point: carla.Transform):
    #try go back to 1000
    if math_utility.sub(spectator.get_location(), spawn_point.location).length() > 1000:
        print('Moving spectator to', spawn_point.location)
        move_spectator_to(spawn_point)
        world.tick()

def __spawn_actor(blueprint, spawn_point: carla.Transform, attach_to: carla.Actor = None):
    setup_spectator(carla.Transform(math_utility.add(spawn_point.location, attach_to.get_location() if attach_to != None else carla.Location())))
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    if isinstance(actor, carla.Vehicle):
        actor.apply_control(carla.VehicleControl(brake=0.5))
    world.tick()
    return actor

old_spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
mid_lane_wp = world.get_map().get_waypoint(carla.Location(x=2390, y=6110, z=187), True)
left_lane_wp = world.get_map().get_waypoint(carla.Location(x=2385, y=6110, z=187), True)

vehicle_list = [
    'vehicle.audi.a2', 
    'vehicle.nissan.micra', 
    'vehicle.audi.tt', 
    'vehicle.mercedes.coupe_2020', 
    'vehicle.bmw.grandtourer', 
    # 'vehicle.harley-davidson.low_rider', 
    'vehicle.ford.ambulance', 
    # 'vehicle.micro.microlino', 
    'vehicle.carlamotors.firetruck', 
    'vehicle.carlamotors.carlacola', 
    'vehicle.carlamotors.european_hgv', 
    'vehicle.ford.mustang', 
    'vehicle.chevrolet.impala', 
    'vehicle.lincoln.mkz_2020', 
    'vehicle.citroen.c3', 
    'vehicle.dodge.charger_police', 
    'vehicle.nissan.patrol', 
    'vehicle.jeep.wrangler_rubicon', 
    'vehicle.mini.cooper_s', 
    'vehicle.mercedes.coupe', 
    'vehicle.dodge.charger_2020', 
    'vehicle.ford.crown', 
    'vehicle.seat.leon', 
    'vehicle.toyota.prius', 
    # 'vehicle.yamaha.yzf', 
    # 'vehicle.kawasaki.ninja', 
    # 'vehicle.bh.crossbike', 
    # 'vehicle.mitsubishi.fusorosa', 
    'vehicle.tesla.model3', 
    # 'vehicle.gazelle.omafiets', 
    'vehicle.tesla.cybertruck', 
    # 'vehicle.diamondback.century', 
    'vehicle.mercedes.sprinter', 
    'vehicle.audi.etron', 
    'vehicle.volkswagen.t2', 
    'vehicle.lincoln.mkz_2017', 
    'vehicle.dodge.charger_police_2020', 
    # 'vehicle.vespa.zx125', 
    'vehicle.mini.cooper_s_2021', 
    'vehicle.nissan.patrol_2021', 
    'vehicle.volkswagen.t2_2021'
]
def spawn(way_point: carla.Waypoint, spawn_distance):
    spawn_point = way_point.next(spawn_distance)[0].transform
    spawn_point.location.z += 2
    # return spawn_vehicle_bp_at('vehicle.tesla.cybertruck', spawn_point)
    return spawn_vehicle_bp_at(rnd.choice(vehicle_list), spawn_point)

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
        tm.set_desired_speed(v[0], rnd.randint(70, 130))
        v[0].set_autopilot(True)
    return lambda: handle_traffic_same_spawn(vehicles)

def handle_traffic(vehicles):
    for v in vehicles:
        try:
            current_wp = world.get_map().get_waypoint(v.get_location(), True)
            current_wp_spawn = get_wp_from_lane_id(current_wp.lane_id)
            if current_wp_spawn.transform.location.distance(v.get_location()) > 1000:
                spawn_point = current_wp_spawn.next(30)[0].transform
                spawn_point.location.z += 2
                v.apply_control(carla.VehicleControl())
                v.set_transform(spawn_point)
        except:
            continue
              

def spawn_traffic(num_vehicle_per_lane: int):
    spawn_distance = 50
    vehicles = []
    for i in range(num_vehicle_per_lane):
            vehicles.append(spawn(mid_lane_wp.get_right_lane(), spawn_distance * (i + 1)))
            vehicles.append(spawn(mid_lane_wp, spawn_distance * (i + 1)))
            vehicles.append(spawn(mid_lane_wp.get_left_lane(), spawn_distance * (i + 1)))
    for v in vehicles:
        tm.set_desired_speed(v, rnd.randint(70, 130))
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

def spawn_vehicle_bp(vehicle, spawn_index=0, transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(vehicle) 
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    return __spawn_actor(vehicle_bp, __transform_vector(spawn_point, transform))

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

def spawn_radar(attach_to, transform=carla.Transform(carla.Location(x=1.2, z=1.6), carla.Rotation(pitch=0)), horizontal_fov = 30, vertical_fov = 30, range=100, points_per_second=100000, sensor_tick = 0):
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
        v.destroy()
    time.sleep(2)    

class VehicleWithRadar:
    def __init__(self, vehicle, acc_info, vehicle_controller = rl_controller.RLController(), show_detection=False, show_range=False, show_filter=False):
        self.vehicle = vehicle
        self.acc_info = acc_info
        self.radar_detection_h_radius = radar_detection_h_radius
        self.radar_detection_v_radius = radar_detection_v_radius
        self.vehicle_controller = vehicle_controller
        self.show_detection = show_detection
        self.show_range = show_range
        self.show_filter = show_filter
        self.vehicle_control = carla.VehicleControl()
        self.min_ttc = self.min_depth = self.relative_velocity = float('inf')
        self.radar = spawn_radar(vehicle, range=radar_range)
        self.radar.listen(self.__radar_callback)
        time.sleep(1)

    def __radar_callback(self, data):
        distances = []
        velocities = []
        ttc = []
        self.show_range and debug_utility.draw_radar_bounding_range(self.radar)
        self.show_filter and debug_utility.draw_radar_point_cloud_range(self.radar, self.radar_detection_h_radius, self.radar_detection_v_radius)
        for detection in data:
            if debug_utility.evaluate_point(self.radar, detection, self.radar_detection_h_radius, self.radar_detection_v_radius):
                self.show_detection and debug_utility.draw_radar_point(self.radar, detection)
                distances.append(detection.depth)
                velocities.append(detection.velocity)
                ttc.append(detection.depth / detection.velocity if abs(detection.velocity) != 0 else float('inf'))
        self.min_ttc = min(ttc) if len(ttc) > 0 else float('inf')
        self.min_depth = distances[ttc.index(self.min_ttc)] if len(distances) > 0 else float('inf')
        self.relative_velocity = velocities[ttc.index(self.min_ttc)] if len(velocities) > 0 else float('inf')

    def compute_control(self):
        self.vehicle_controller.apply_control(self)
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
    
def print_text_to_screen(display, text, position, color, right=False):
        font = pygame.font.Font(None, 36)  # You can choose the font and size
        text_surface = font.render(text, True, color)  # White color
        if right:
            position = (display.get_width() - text_surface.get_width() - position[0], position[1])
        display.blit(text_surface, position)

class FadingText(object):
    def __init__(self, font):
        self.font = font
        self.text_texture = self.font.render("", True, (255, 0, 0))
        self.seconds_left = 0

    def set_text(self, text, color=(255, 0, 0), seconds=2.0):
        self.text_texture = self.font.render(text, True, color)
        self.seconds_left = seconds

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.text_texture.set_alpha(500.0 * self.seconds_left)

    def render(self, display: pygame.Surface):
        display.blit(self.text_texture, ((display.get_width()-self.text_texture.get_width())/2, display.get_height()-40))

class CameraManager(object):
    def __init__(self, display, parent_actor, transform_index = 0):
        self.sensor = None
        self.surface = None
        self.display = display
        self.last_width = display.get_width()
        self.last_height = display.get_height()
        self._parent = parent_actor
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-7.5, z=2.8), carla.Rotation(pitch=-10)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = transform_index
        self.sensors = ['sensor.camera.rgb', cc.Raw, 'Camera RGB']
        self.sensors.append(self._parent.get_world().get_blueprint_library().find(self.sensors[0]))
    
        self.__spawn_camera()    

    def __spawn_camera(self):
        self.sensors[-1].set_attribute('image_size_x', str(self.display.get_width()))
        self.sensors[-1].set_attribute('image_size_y', str(self.display.get_height()))
        self.last_width = self.display.get_width()
        self.last_height = self.display.get_height()
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

    def render(self, display, program_info):
        if self.surface is not None:
            scaled_surface = pygame.transform.scale(self.surface, display.get_size())
            display.blit(scaled_surface, (0, 0))
        if self.last_width != display.get_width() or self.last_height != display.get_height():
            self.__spawn_camera()
           
        print_text_to_screen(display, f"Throttle: {program_info.ego_control.throttle}", (10, 10), (255, 255, 255))
        print_text_to_screen(display, f"Brake: {program_info.ego_control.brake}", (10, 50), (255, 255, 255))
        print_text_to_screen(display, f"Steer: {program_info.ego_control.steer}", (10, 90), (255, 255, 255))
        print_text_to_screen(display, f"Reverse: {program_info.ego_control.reverse}", (10, 130), (255, 255, 255))
        print_text_to_screen(display, f"CC: {program_info.ego_vehicle.acc_info.is_active()}", (10, 170), (255, 255, 255))
        print_text_to_screen(display, f"Distance Mode: {((program_info.ego_vehicle.acc_info.min_permitted_offset % 7)+1)}", (10, 210), (255, 255, 255))
        print_text_to_screen(display, f"Target Velocity: {program_info.ego_vehicle.acc_info.target_velocity}", (10, 250), (255, 255, 255))
        print_text_to_screen(display, f"Ego Velocity: {program_info.ego_velocity}", (10, 290), (255, 255, 255))
        other_vehicle_velocity = program_info.ego_velocity + program_info.obstacle_relative_velocity
        print_text_to_screen(display, f"Obstacle Velocity: {other_vehicle_velocity}", (10, 330), (255, 255, 255))
        print_text_to_screen(display, f"Min TTC: {program_info.ego_vehicle.min_ttc}", (10, 370), (255, 255, 255))
        print_text_to_screen(display, f"Min Depth: {program_info.ego_vehicle.min_depth}", (10, 410), (255, 255, 255))
        if program_info.other_vehicle != None:
            print_text_to_screen(display, f"CC: {program_info.other_vehicle.acc_info.is_active()}", (10, display.get_height() - 130), (255, 255, 255))
            print_text_to_screen(display, f"Target Velocity: {program_info.other_vehicle.acc_info.target_velocity}", (10, display.get_height() - 90), (255, 255, 255))
            print_text_to_screen(display, f"Other Vehicle Velocity: {program_info.other_vehicle.vehicle.get_velocity().length() * 3.6}", (10, display.get_height() - 50), (255, 255, 255))


    def _parse_image(self, image):
        image.convert(self.sensors[1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class CollisionSensor(object):
    def __init__(self, parent_actor, fading_text, program_info):
        self.sensor = None
        self._parent = parent_actor
        self.fading_text = fading_text
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        self.sensor.listen(self._on_collision)
        self.program_info = program_info

    def _on_collision(self, event):
        # Display the collision on the screen
        self.fading_text.set_text(f"Collision with {get_actor_display_name(event.other_actor)}", seconds=2.0)
        self.program_info.send_info and server.send_data(f"Collision with {get_actor_display_name(event.other_actor)}")