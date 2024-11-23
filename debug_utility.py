import numpy as np
import math
import carla

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

def __get_point_at(radar: carla.Actor, at: int):
    radar_location = radar.get_location()
    radar_f_vector = radar.get_transform().get_forward_vector()
    return __vector_scalar_op(radar_location, radar_f_vector, lambda a, b: a + at * b)

def draw_radar_point(radar: carla.Actor, point: carla.RadarDetection):
    debug = radar.get_world().debug
    radar_location = radar.get_location()
    shifted_max_range_point = to_np(__sub(__get_point_at(radar, point.depth), radar_location))
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, point.azimuth, point.altitude)),radar_location), life_time=0.1)
    #radar.get_world().tick()

def draw_radar_bounding_box(radar: carla.Actor):
    debug = radar.get_world().debug
    radar_h_fov = math.radians(int(radar.attributes.get('horizontal_fov')))
    radar_range = int(radar.attributes.get('range'))
    radar_v_fov = math.radians(int(radar.attributes.get('vertical_fov')))
    radar_location = radar.get_location()

    shifted_max_range_point = to_np(__sub(__get_point_at(radar, radar_range), radar_location))
    debug.draw_line(radar_location, __add(from_np(shifted_max_range_point), radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, radar_h_fov, radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, radar_h_fov, -radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, -radar_h_fov, radar_v_fov)),radar_location), life_time=0.1)
    debug.draw_line(radar_location, __add(from_np(__rotate(shifted_max_range_point, -radar_h_fov, -radar_v_fov)),radar_location), life_time=0.1)
    #radar.get_world().tick()