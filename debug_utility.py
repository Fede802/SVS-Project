import numpy as np
import math
import carla, time
from typing import List

def __yaw_matrix(yaw = 0):
    return np.array([[math.cos(yaw), -math.sin(yaw), 0],
                      [math.sin(yaw), math.cos(yaw), 0],
                      [0, 0, 1]])

def __pitch_matrix(pitch = 0):
    return np.array([[math.cos(pitch), 0, math.sin(pitch)],
                      [0, 1, 0],
                      [-math.sin(pitch), 0, math.cos(pitch)]])

def __roll_matrix(roll = 0):
    return np.array([[1, 0, 0],
                      [0, math.cos(roll), -math.sin(roll)],
                      [0, math.sin(roll), math.cos(roll)]])

def __rotate(point3d = np.array([0,0,0]), yaw = 0, pitch = 0, roll = 0):
    return np.dot(__yaw_matrix(yaw) @ __pitch_matrix(pitch) @ __roll_matrix(roll), point3d)

def __vector_scalar_op(p3d: carla.Vector3D, transaltion: carla.Vector3D, op):
    return carla.Vector3D(op(p3d.x, transaltion.x), op(p3d.y, transaltion.y), op(p3d.z, transaltion.z))

def __add(p3d: carla.Vector3D, transaltion: carla.Vector3D):
    return __vector_scalar_op(p3d, transaltion, lambda a, b: a + b)

def __sub(p3d: carla.Vector3D, transaltion: carla.Vector3D):
    return __vector_scalar_op(p3d, transaltion, lambda a, b: a - b)

def to_np(v3d: carla.Vector3D):
    return np.array([v3d.x, v3d.y, v3d.z])

def from_np(v3d: np.ndarray):
    v3d = v3d.reshape(-1)
    return carla.Vector3D(v3d[0], v3d[1],v3d[2])

def get_point_at_general(start: carla.Vector3D, radar: carla.Actor, at: int):
    radar_f_vector = radar.get_transform().get_forward_vector()
    #print(radar_f_vector.length())
    return __vector_scalar_op(start, radar_f_vector, lambda a, b: a + at * b)
def get_point_at(radar: carla.Actor, at: int):
    radar_location = radar.get_location()
    radar_f_vector = radar.get_transform().get_forward_vector()
    #print(radar_f_vector.length())
    return __vector_scalar_op(radar_location, radar_f_vector, lambda a, b: a + at * b)

def draw_radar_point(radar: carla.Actor, point: carla.RadarDetection, color = carla.Color(0, 255, 0)):
    debug = radar.get_world().debug
    radar_location = radar.get_location()
    shifted_max_range_point = to_np(__sub(get_point_at(radar, point.depth), radar_location))

    azi_angle = point.azimuth + math.radians(radar.get_transform().rotation.yaw)
    alti_angle = point.altitude + math.radians(radar.get_transform().rotation.pitch)


    x = point.depth * math.cos(azi_angle) * math.cos(alti_angle)
    y = point.depth * math.sin(azi_angle) * math.cos(alti_angle)
    z = -point.depth * math.cos(azi_angle) * math.sin(alti_angle)
    #print("[DEBUG]: x=", x, "y=", y, "z=", z, "alt = ", __rotate(shifted_max_range_point, point.azimuth, point.altitude))
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, point.azimuth, point.altitude)),radar_location), life_time=0.1, color=color)
    #radar.get_world().tick()

def draw_radar_bounding_box(radar: carla.Actor):
    debug = radar.get_world().debug
    radar_h_fov = math.radians(int(radar.attributes.get('horizontal_fov')))
    radar_range = int(radar.attributes.get('range'))
    radar_v_fov = math.radians(int(radar.attributes.get('vertical_fov')))
    radar_location = radar.get_location()

    shifted_max_range_point = to_np(__sub(get_point_at(radar, radar_range), radar_location))
    debug.draw_line(radar_location, __add(from_np(shifted_max_range_point), radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, radar_h_fov, radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, radar_h_fov, -radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, -radar_h_fov, radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, -radar_h_fov, -radar_v_fov)),radar_location), life_time=0.1)
    #radar.get_world().tick()

def __vector_scalar_op(p3d: carla.Vector3D, transaltion: carla.Vector3D, op):
    return carla.Vector3D(op(p3d.x, transaltion.x), op(p3d.y, transaltion.y), op(p3d.z, transaltion.z))

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm  

def get_plane_base_vectors(radar: carla.Actor):
    debug = radar.get_world().debug
    normal_vector = to_np(radar.get_transform().get_forward_vector())
    temp_v = np.array([1,0,0])
    if not np.any(np.cross(normal_vector, temp_v)):
        temp_v = np.array([0,1,0])

    v1 = normalize(np.cross(normal_vector, temp_v)) 
    v2 = normalize(np.cross(normal_vector, v1))  
    v1 = from_np(v1)
    v2 = from_np(v2)
    h = 1
    vn = carla.Vector3D(x = 0, y = 0, z = 0)
    
    p1 = __add(__add(radar.get_location(), __vector_scalar_op(vn, v1, lambda a, b: h * b)),  __vector_scalar_op(vn, v2, lambda a, b: h * b))
    p2 = __sub(__add(radar.get_location(), __vector_scalar_op(vn, v1, lambda a, b: h * b)),  __vector_scalar_op(vn, v2, lambda a, b: h * b))
    p3 = __sub(__sub(radar.get_location(), __vector_scalar_op(vn, v1, lambda a, b: h * b)),  __vector_scalar_op(vn, v2, lambda a, b: h * b))    
    p4 = __add(__sub(radar.get_location(), __vector_scalar_op(vn, v1, lambda a, b: h * b)),  __vector_scalar_op(vn, v2, lambda a, b: h * b))

    p5 = get_point_at_general(p1, radar, 50)
    p6 = get_point_at_general(p2, radar, 50)
    p7 = get_point_at_general(p3, radar, 50)
    p8 = get_point_at_general(p4, radar, 50)
    
    debug.draw_line(p1, p5, life_time=0.1)
    debug.draw_line(p2, p6, life_time=0.1)
    debug.draw_line(p3, p7, life_time=0.1)
    debug.draw_line(p4, p8, life_time=0.1)
   

    

def evaluate_point(radar: carla.Actor, point: carla.RadarDetection):
    
    x_min = 0
    y_min = -2
    z_min = -1.8
    x_max = 100
    y_max = 2
    z_max = 2

    azi_angle = point.azimuth 
    alti_angle = point.altitude 


    x = point.depth * math.cos(azi_angle) * math.cos(alti_angle)
    y = point.depth * math.sin(azi_angle) * math.cos(alti_angle)
    z = -point.depth * math.cos(azi_angle) * math.sin(alti_angle) +2

    debug = radar.get_world().debug
    radar_location = radar.get_location()

    #if x_min < x < x_max and y_min < y < y_max and z_min < z < z_max:
        #print(radar.get_transform())
    #print(x, y, z)
    #debug.draw_line(radar_location, __add(carla.Vector3D(x,y,z),radar_location), life_time=1, color=carla.Color(255, 255, 0))
    #time.sleep(1)

    return x_min < x < x_max and y_min < y < y_max and z_min < z < z_max
