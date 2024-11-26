import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility
import carla_utility
import plot_utility

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses
class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '160')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(lambda data: self.save_lidar_image(data, point_list))

            return lidar
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '10')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_lidar_image(self, image, point_list):
        t_start = self.timer.time()
        """Prepares a point cloud with intensity
        colors ready to be consumed by Open3D"""
        data = np.copy(np.frombuffer(image.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the intensity and compute a color for it
        intensity = data[:, -1]
        intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
        int_color = np.c_[
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

        # Isolate the 3D data
        points = data[:, :-1]

        # We're negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        points[:, :1] = -points[:, :1]

        # # An example of converting points from sensor to vehicle space if we had
        # # a carla.Transform variable named "tran":
        # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
        # points = np.dot(tran.get_matrix(), points.T).T
        # points = points[:, :-1]

        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)
        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


client = carla.Client('localhost', 2000)
print(client.get_available_maps())
#time.sleep(1000)
#3 bella dritta, 4 anche meglio


world = client.get_world()
spectator = world.get_spectator()

f = open("log.txt", "a")
point_list = o3d.geometry.PointCloud()

#vPlot = plot_utility.RealTimePlotApp("Velocity")
#aPlot = plot_utility.RealTimePlotApp("Acceleration")

target_velocity = 70
def lidar_callback(data: carla.LidarMeasurement):
    print("-------------LIDAR MESUREMENT-------------")
    print(data)
    f.write("-------------LIDAR MESUREMENT-------------\n")
    f.write(str(data)+"\n")
    f.write(str(len(data))+"\n")
    for location in data:
        #f.write(str(location)+"\n")
        print(location)
    f.write("-----------END LIDAR MESUREMENT------------\n")    
    print("-----------END LIDAR MESUREMENT------------")    
#TTC = (distance / relative_velocity) if relative_velocity != 0 else 9999
# Simple callback function to print the number of detections
min_ttc = float('inf')
def handle_measurement(data: carla.RadarMeasurement, radar: carla.Actor):
    global min_ttc, min_distance
    min_ttc = float('inf')
    
    for detection, i in zip(data, range(len(data))):
        print(detection)
        absolute_speed = abs(detection.velocity)
        #debug_utility.draw_radar_point(radar, detection)
        # Calculate TTC
        if absolute_speed != 0:
            ttc = detection.depth / absolute_speed
            if ttc < min_ttc:
                min_ttc = ttc

pygame.init()
pygame.display.set_mode((400, 300))

for v in world.get_actors().filter('vehicle.*'):
    v.destroy()

#ego_vehicle = carla_utility.spawn_vehicle(world=world, vehicle_index=15, transform=carla.Transform(rotation = carla.Rotation(yaw=45)))
#ego_vehicle = carla_utility.spawn_vehicle(world=world)
#ego_vehicle = carla_utility.spawn_veichle_at(world=world, vehicle_index=15, spawn_point=carla.Transform(carla.Location(x=2.484849, y=-170.415253, z=2.900956)))
#ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=380.786957, y=31.491543, z=13.309415), carla.Rotation(yaw = 180)))
ego_vehicle = carla_utility.spawn_veichle_bp_at(world=world, vehicle='vehicle.tesla.cybertruck', spawn_point=carla.Transform(carla.Location(x=408.760681, y=-71.501823, z=7.077427), carla.Rotation(yaw = 102)))
print(ego_vehicle.get_location())
#other_vehicle = carla_utility.spawn_vehicle(world=world, transform=carla.Transform(carla.Location(x=50)))
other_vehicle = carla_utility.spawn_veichle_bp_in_front_of(world, ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=10)

#radar = carla_utility.spawn_radar(world, ego_vehicle, range=54)
#radar.listen(lambda data: handle_measurement(data, radar))
#lidar = carla_utility.spawn_lidar(world, ego_vehicle)
#lidar.listen(lambda data: lidar_callback(data))

video_output = np.zeros((600, 800, 4), dtype=np.uint8)
def camera_callback(image):
    global video_output
    video_output = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))





def compute_controls(velocita_corrente, velocita_target, ttc, k_p=0.05):
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
    
    if min_ttc > 0 and min_ttc < float('inf'):
        print(min_ttc)
        brake = 1.0
    else:    
        if errore_vel > 0:
            throttle = 1.0 * errore_vel_perc
        else:
            brake = 1.0 * errore_vel_perc    
        # Accelerazione
     #   throttle = min(0.5 + k_p * errore_vel, 1.0)
    #elif errore_vel < 0:
        # Decelerazione
     #   brake = min(-k_p * errore_vel, 1.0)

    #if brake > 0:
     #   throttle = 0.0
    
    # TTC Safety Override
    #if ttc < 1.5:
     #   brake = 1.0
      #  throttle = 0.0
    
    return throttle, brake

carla_utility.move_spectator_to(world, ego_vehicle.get_transform())
show_in_carla = False
show_in_camera = True
running = True
cruise_control = False
lastUpdate = 0

vis = o3d.visualization.Visualizer()
vis.create_window(
    window_name='Carla Lidar',
    width=960,
    height=540,
    left=480,
    top=270)
vis.get_render_option().background_color = [0.05, 0.05, 0.05]
vis.get_render_option().point_size = 1
vis.get_render_option().show_coordinate_frame = True

add_open3d_axis(vis)

frame = 0

display_manager = DisplayManager(grid_size=[2, 3], window_size=[800,600])

# Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
# and assign each of them to a grid position, 
SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                ego_vehicle, {}, display_pos=[0, 0])
SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                ego_vehicle, {}, display_pos=[0, 1])
SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                ego_vehicle, {}, display_pos=[0, 2])
SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                ego_vehicle, {}, display_pos=[1, 1])

SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
                ego_vehicle, {'channels' : '64', 'range' : '160',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 0])
SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
                ego_vehicle, {'channels' : '64', 'range' : '30', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])


if show_in_camera:
    camera = carla_utility.spawn_camera(world=world, attach_to=ego_vehicle, transform=carla.Transform(carla.Location(x=-6, z=5), carla.Rotation(pitch=-30)))
    camera.listen(lambda image: camera_callback(image))
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

try:
    while running:
        control = carla.VehicleControl()
        other_vehicle.apply_control(carla.VehicleControl(brake=0.5))
        #other_vehicle.apply_control(carla.VehicleControl(throttle=0.5))
        #control.throttle = 1.0

        #debug_utility.draw_radar_bounding_box(radar)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
                    running = False
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            control.throttle = 0.5
        if keys[pygame.K_s]:
            control.throttle = 0.5
            control.reverse = True
        if keys[pygame.K_a]:
            control.steer = -0.38
        if keys[pygame.K_d]:
            control.steer = 0.38
        if keys[pygame.K_e]:
            control.brake = 0.5
        if keys[pygame.K_r]:
            cruise_control = True
        if keys[pygame.K_p]:
            cruise_control = False 
        if cruise_control:              
            throttle, brake = compute_controls(ego_vehicle.get_velocity().length(), target_velocity, min_ttc)
            control = carla.VehicleControl(throttle=throttle, brake=brake)
        print(control)    
        ego_vehicle.apply_control(control)
        display_manager.render()

        if show_in_carla:
            carla_utility.move_spectator_to(world, ego_vehicle.get_transform())
        if show_in_camera:
            cv2.imshow('RGB Camera', video_output)
        pygame.display.flip()
        #print("Actor transform: ", ego_vehicle.get_transform())
        #print("Actor forward vector (direction):", ego_vehicle.get_transform().get_forward_vector())
        #print("Actor control: ",ego_vehicle.get_control().throttle," ", ego_vehicle.get_control().brake," Actor velocity: ", ego_vehicle.get_velocity(),", ",ego_vehicle.get_velocity().length(),"m/s, ",ego_vehicle.get_velocity().length()*3.6,"km/h")
        
        if(time.time() - lastUpdate > 0.2):
            #vPlot.add_value(ego_vehicle.get_velocity().length()*3.6)
            #aPlot.add_value(ego_vehicle.get_acceleration().length())
            lastUpdate = time.time()

        if frame == 2:
            vis.add_geometry(point_list)
        vis.update_geometry(point_list)

        vis.poll_events()
        vis.update_renderer()
        # # This can fix Open3D jittering issues:
        time.sleep(0.005)
        world.tick()


        
        frame += 1    
        world.tick()
except KeyboardInterrupt:
    pass 
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    #radar.stop()
    #radar.destroy()
    f.close()
    camera.destroy()
    #lidar.destroy()
    vis.destroy_window()

    ego_vehicle.destroy()
    other_vehicle.destroy()     