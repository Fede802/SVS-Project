import carla, time, pygame, math, random, cv2
import open3d as o3d
from matplotlib import cm
import numpy as np
import debug_utility

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()
spectator = world.get_spectator()

def move_spectator_to(transform, distance=5.0, x=0, y=0, z=4, yaw=0, pitch=-30, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    
    spectator_transform = carla.Transform(back_location, transform.rotation)
    spectator.set_transform(spectator_transform)

def spawn_vehicle(vehicle_index=0, spawn_index=0, pattern='vehicle.*', transform = carla.Transform()):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    spawn_point.location.x += transform.location.x
    spawn_point.location.y += transform.location.y
    spawn_point.location.z += transform.location.z
    spawn_point.rotation.pitch += transform.rotation.pitch
    spawn_point.rotation.roll += transform.rotation.roll
    spawn_point.rotation.yaw += transform.rotation.yaw
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

def draw_on_screen(world, transform, content='O', color=carla.Color(0, 255, 0), life_time=20):
    world.debug.draw_string(transform.location, content, color=color, life_time=life_time)

def spawn_camera(attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    #camera_bp.set_attribute('fov', '120')
    #camera_bp.set_attribute('sensor_tick', '0')
    camera = world.spawn_actor(camera_bp, transform, attach_to=attach_to)
    return camera

def spawn_radar(attach, transform=carla.Transform(carla.Location(x=1.2, z=1.2)), range=20, points_per_second=10):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '30')
    radar_bp.set_attribute('vertical_fov', '30')
    #radar_bp.set_attribute('range', str(100.0))
    #print("sdhgakjfhjd")
    #print(type(radar_bp.get_attribute('range')))
    #print(radar_bp.get_attribute('range'))
    radar_bp.set_attribute('range', str(range))
    #print(type(radar_bp.get_attribute('range')))
    #print(radar_bp.get_attribute('range'))
    #radar_bp.set_attribute('points_per_second', str(points_per_second))
    radar = world.spawn_actor(radar_bp, transform, attach_to=attach)
    return radar

#TTC = (distance / relative_velocity) if relative_velocity != 0 else 9999
def type_detection(data) -> carla.RadarDetection:
    return data
    

# Simple callback function to print the number of detections
def handle_measurement(data: carla.RadarMeasurement, vehicle: carla.Vehicle):
    print(len(data))
    print(data.get_detection_count())
    for m in data:
        newM = type_detection(m)
        print("data: ",newM)
        print("DIO LAIDO ", newM.depth)



pygame.init()
pygame.display.set_mode((400, 300))

for v in world.get_actors().filter('vehicle.*'):
    v.destroy()

ego_vehicle = spawn_vehicle(vehicle_index=15, transform=carla.Transform(rotation = carla.Rotation(yaw=45)))
#trasnform ego_vehicle = spawn_vehicle(vehicle_index=15)
time.sleep(1)
radar = spawn_radar(attach=ego_vehicle)
time.sleep(1)
#print(radar.attributes.get('horizontal_fov'))
#print(radar.attributes.get('vertical_fov'))
#print(radar.get_transform())
#print(radar.get_location())
fwv = radar.get_transform().get_forward_vector()
radar_range = radar.attributes.get('range')
radar_location = radar.get_location()
radar_transform = radar.get_transform()
radar_f_vector = radar_transform.get_forward_vector()
print(radar_range)
print(radar_f_vector.x)
print(int(radar_range)*radar_f_vector.x)
#print(fwv)
#print(fwv.length())
#print(radar.bounding_box)
#print(ego_vehicle.get_transform())
#radar.listen(lambda data: handle_measurement(data, ego_vehicle))
other_vehicle = spawn_vehicle(transform=carla.Transform(carla.Location(x=50)))
debug_utility.draw_radar_bounding_box(radar)
#camera = spawn_camera(attach_to=ego_vehicle, transform=carla.Transform(carla.Location(x=-6, z=5), carla.Rotation(pitch=-30)))

#point_list = o3d.geometry.PointCloud()
#radar_list = o3d.geometry.PointCloud()

#camera_data = {'image': np.zeros((800,600,4))}

#video_output = np.zeros((600, 800, 4), dtype=np.uint8)
#def camera_callback(image):
    #global video_output
    #video_output = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

#camera.listen(lambda image: camera_callback(image))
#camera.listen(lambda image: camera_callback(image, camera_data))
#radar.listen(lambda data: radar_callback(data, radar_list))
#vehicle.set_autopilot(True)
#control = carla.VehicleControl(throttle=1000)
#vehicle.apply_control(control)

#cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
#cv2.imshow('RGB Camera', camera_data['image'])
#cv2.waitKey(1)

#vis = o3d.visualization.Visualizer()
#vis.create_window(
    #window_name ='Carla',
    #width=960,
    #height=540,
    #left=480,
    #top=270)

#vis.get_render_option().background_color = [0.05, 0.05, 0.05]
#vis.get_render_option().point_size = 1
#vis.get_render_option().show_coordinate_frame = True
#add_open3d_axis(vis)

#frame = 0

running = True
moving_forward = False
move_spectator_to(ego_vehicle.get_transform())

try:
    while running:
        current_location = ego_vehicle.get_location()
        #move_spectator_to(ego_vehicle.get_transform())
        debug_utility.draw_radar_bounding_box(radar)
        time.sleep(0.001)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                key = event.key
                if key == pygame.QUIT or key == pygame.K_ESCAPE:
                    running = False
                elif key == pygame.K_w:
                    current_location.x += 2
                    #world.tick()
                    moving_forward = True
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                elif key == pygame.K_s:
                    current_location.x -= 2
                    ego_vehicle.apply_control(carla.VehicleControl(brake=0.1))
                    #world.tick()
                    moving_forward = False

        
        
        world.tick()
        
        pygame.display.flip()


        #print("Actor transform: ", ego_vehicle.get_transform())
        #print("Actor forward vector (direction):", ego_vehicle.get_transform().get_forward_vector())
        #print("Actor velocity: ", ego_vehicle.get_velocity(),", ",ego_vehicle.get_velocity().length(),"m/s, ",ego_vehicle.get_velocity().length()*3.6,"km/h")
        #cv2.imshow('RGB Camera', video_output)
        #move_spectator_to(ego_vehicle.get_transform())
except KeyboardInterrupt:
    pass 
finally:
    pygame.quit()
    cv2.destroyAllWindows()
    radar.destroy()
    #camera.stop()
    #camera.destroy()
    ego_vehicle.destroy()
    other_vehicle.destroy()
    #vis.destroy_window()        