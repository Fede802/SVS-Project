import math
import numpy as np
import carla

def __to_np(v3d: carla.Vector3D):
    return np.array([v3d.x, v3d.y, v3d.z])

def __from_np(v3d: np.ndarray):
    v3d = v3d.reshape(-1)
    return carla.Vector3D(v3d[0], v3d[1],v3d[2])

def __normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm 

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

def rotate(point3d = carla.Vector3D(), yaw = 0, pitch = 0, roll = 0):
    return __from_np(np.dot(__yaw_matrix(yaw) @ __pitch_matrix(pitch) @ __roll_matrix(roll), __to_np(point3d)))

def vector_scalar_op(op, p1: carla.Vector3D, p2 = carla.Vector3D()):
    return carla.Vector3D(op(p1.x, p2.x), op(p1.y, p2.y), op(p1.z, p2.z))

def add(p1: carla.Vector3D, p2: carla.Vector3D):
    return vector_scalar_op(lambda a, b: a + b, p1, p2)

def sub(p1: carla.Vector3D, p2: carla.Vector3D):
    return vector_scalar_op(lambda a, b: a - b, p1, p2)

def get_plane_base_versor_from_normal_versor(normal_versor: carla.Vector3D):
    normal_versor = __to_np(normal_versor)
    temp_versor = np.array([1,0,0])

    if not np.any(np.cross(normal_versor, temp_versor)):
        temp_versor = np.array([0,1,0])

    base_versor1 = __normalize(np.cross(normal_versor, temp_versor)) 
    base_versor2 = __normalize(np.cross(normal_versor, base_versor1))

    return (__from_np(base_versor1), __from_np(base_versor2))
