import numpy as np
import math
import carla
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
    radar.get_transform().get_inverse_matrix()
    #print(radar_f_vector.length())
    return __vector_scalar_op(radar_location, radar_f_vector, lambda a, b: a + at * b)

def draw_radar_point(radar: carla.Actor, point: carla.RadarDetection):
    debug = radar.get_world().debug
    radar_location = radar.get_location()
    shifted_max_range_point = to_np(__sub(get_point_at(radar, point.depth), radar_location))
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, point.azimuth, point.altitude)),radar_location), life_time=0.1)
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
    print("------DATA-----")
    debug.draw_line(p1, p5, life_time=0.1)
    debug.draw_line(p2, p6, life_time=0.1)
    debug.draw_line(p3, p7, life_time=0.1)
    debug.draw_line(p4, p8, life_time=0.1)
    print(p1)
    matrix = radar.get_transform().get_inverse_matrix()
    cirm = np.array([[matrix[0][0], matrix[0][1], matrix[0][2]],[matrix[1][0], matrix[1][1], matrix[1][2]],[matrix[2][0], matrix[2][1], matrix[2][2]]])
    print(np.dot(cirm, to_np(p1)))
    print("------end DATA-----")
    p1 = from_np(np.dot(cirm, to_np(p1)))
    p2 = from_np(np.dot(cirm, to_np(p2)))
    p3 = from_np(np.dot(cirm, to_np(p3)))
    p4 = from_np(np.dot(cirm, to_np(p4)))

    p5 = from_np(np.dot(cirm, to_np(p5)))
    p6 = from_np(np.dot(cirm, to_np(p6)))
    p7 = from_np(np.dot(cirm, to_np(p7)))
    p8 = from_np(np.dot(cirm, to_np(p8)))
    debug.draw_line(p1, p5, life_time=0.1)
    debug.draw_line(p2, p6, life_time=0.1)
    debug.draw_line(p3, p7, life_time=0.1)
    debug.draw_line(p4, p8, life_time=0.1)

    debug.draw_line(p1, p2, life_time=0.1)
    debug.draw_line(p2, p3, life_time=0.1)
    debug.draw_line(p3, p4, life_time=0.1)
    debug.draw_line(p4, p1, life_time=0.1)
    
    

def get_inverse_rotation_matrix_and_vertices(radar: carla.Actor):    
    normal_vector = to_np(radar.get_transform().get_forward_vector())
    temp_v = np.array([1,0,0])
    if not np.any(np.cross(normal_vector, temp_v)):
        temp_v = np.array([0,1,0])
    matrix = radar.get_transform().get_inverse_matrix()
    r = np.array([[matrix[0][0], matrix[0][1], matrix[0][2]],[matrix[1][0], matrix[1][1], matrix[1][2]],[matrix[2][0], matrix[2][1], matrix[2][2]]])
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

    

    return (r, np.dot(r,to_np(p1)), np.dot(r,to_np(p2)), np.dot(r,to_np(p3)), np.dot(r,to_np(p4)), np.dot(r,to_np(p5)), np.dot(r,to_np(p6)), np.dot(r,to_np(p7)), np.dot(r,to_np(p8)))

def evaluate_point(radar: carla.Actor, point: carla.Vector3D):





    r, p1, p2, p3, p4, p5, p6, p7, p8 = get_inverse_rotation_matrix_and_vertices(radar)
    x_min = min(p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0] )
    y_min = min(p1[1], p2[1], p3[1], p4[1], p5[1], p6[1], p7[1], p8[1] )
    z_min = min(p1[2], p2[2], p3[2], p4[2], p5[2], p6[2], p7[2], p8[2] )

    x_max = max(p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0] )
    y_max = max(p1[1], p2[1], p3[1], p4[1], p5[1], p6[1], p7[1], p8[1] )
    z_max = max(p1[2], p2[2], p3[2], p4[2], p5[2], p6[2], p7[2], p8[2] )

    point = get_point_at(radar, point.depth)
    rp = np.dot(r, to_np(point))

    return x_min < rp[0] < x_max and y_min < rp[1] < y_max and z_min < rp[2] < z_max
