import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
import numpy as np
from collections import deque
import carla_utility, time, carla

class PID:
    def __init__(self, buffer_size, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = -1
        self.e_buffer = deque(maxlen=buffer_size) if buffer_size != None and buffer_size > 0 else deque()

    def compute_control(self, target, current):
        e = target - current
        self.e_buffer.append(e)
        if self.dt == -1:
            self.dt = time.time()
            ie = 0.0
            de = 0.0
        else:
            self.dt = time.time() - self.dt    
            ie = np.sum(self.e_buffer) * self.dt
            de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt #don't needed -> if len(self.e_buffer) > 1 else 0.0

        return np.clip((self.kp * e) + (self.kd * de) + (self.ki * ie), 0.0, 1.0)
    
class PIDController:
    def __init__(self, buffer_size = 42, kp_velocity = 0.2, ki_velocity = 0.2, kd_velocity = 0.01, 
                kp_distance = 1.6, ki_distance = 0.01, kd_distance = 0.7):
        self.pid_velocity = PID(buffer_size, kp_velocity, ki_velocity, kd_velocity)
        self.pid_distance = PID(buffer_size, kp_distance, ki_distance, kd_distance)

    def apply_control(self, control_info, current_velocity, min_depth):
        min_permitted_distance = carla_utility.compute_security_distance(current_velocity) + control_info.min_permitted_offset
        distance_error = min_depth - min_permitted_distance
        
        if distance_error > 0:
            return carla.VehicleControl(throttle = self.pid_velocity.compute_control(control_info.target_velocity, current_velocity))
        else:
            return carla.VehicleControl(brake = self.pid_distance.compute_control(min_permitted_distance, min_depth))