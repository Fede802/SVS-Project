import carla, time, debug_utility, math_utility, re, pygame, numpy as np
from carla import ColorConverter as cc

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
    weather_index += -1 if reverse else 1
    weather_index %= len(weather_presets)
    preset = weather_presets[weather_index]
    world.set_weather(preset[0])

def __spawn_actor(blueprint, spawn_point: carla.Transform, attach_to: carla.Actor = None):
    __setup_spectactor(carla.Transform(math_utility.add(spawn_point.location, attach_to.get_location() if attach_to != None else carla.Location())))
    actor = world.spawn_actor(blueprint, spawn_point, attach_to)
    time.sleep(10)
    return actor

def __transform_vector(point: carla.Transform, transform: carla.Transform):
    point.location.x += transform.location.x
    point.location.y += transform.location.y
    point.location.z += transform.location.z
    point.rotation.pitch += transform.rotation.pitch
    point.rotation.roll += transform.rotation.roll
    point.rotation.yaw += transform.rotation.yaw
    return point

def move_spectator_to(to, distance=5.0, transform = carla.Transform(carla.Location(x = 5, z=5), carla.Rotation(pitch=-10))):
    spectator_transform = carla.Transform(carla.Location(to.location - to.get_forward_vector() * distance), to.rotation)
    spectator.set_transform(__transform_vector(spectator_transform, transform))
    
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

def __setup_spectactor(spawn_point: carla.Transform):
    #try go back to 1000
    if math_utility.sub(spectator.get_location(), spawn_point.location).length() > 2000:
        spectator.set_transform(spawn_point)
        time.sleep(10)

def setup_spectator(world: carla.World, spawn_point: carla.Transform):
    spectator = world.get_spectator()
    __setup_spectactor(spectator, spawn_point)        

def compute_security_distance(velocity):
    return (velocity // 10) ** 2

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
    
    def __print_text_to_screen(self, display, text, position, color):
        font = pygame.font.Font(None, 36)  # You can choose the font and size
        text_surface = font.render(text, True, color)  # White color
        display.blit(text_surface, position)

    def render(self, display, control_info, ego_veichle: carla.Vehicle):
        if self.surface is not None:
            scaled_surface = pygame.transform.scale(self.surface, display.get_size())
            display.blit(scaled_surface, (0, 0))
        self.__print_text_to_screen(display, f"Throttle: {control_info.ego_control.throttle}", (10, 10), (255, 255, 255))
        self.__print_text_to_screen(display, f"Brake: {control_info.ego_control.brake}", (10, 50), (255, 255, 255))
        self.__print_text_to_screen(display, f"Steer: {control_info.ego_control.steer}", (10, 90), (255, 255, 255))
        self.__print_text_to_screen(display, f"Hand Brake: {control_info.ego_control.hand_brake}", (10, 130), (255, 255, 255))
        self.__print_text_to_screen(display, f"PID CC: {control_info.cc()}", (10, 170), (255, 255, 255))
        self.__print_text_to_screen(display, f"Min Permitted Distance: {control_info.min_permitted_offset}", (10, 210), (255, 255, 255))
        self.__print_text_to_screen(display, f"Target Velocity: {ego_veichle.get_velocity().length() * 3.6}", (10, 250), (255, 255, 255))
        self.__print_text_to_screen(display, f"Target Velocity: {control_info.target_velocity}", (10, 290), (255, 255, 255))

    def _parse_image(self, image):
        image.convert(self.sensors[1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))