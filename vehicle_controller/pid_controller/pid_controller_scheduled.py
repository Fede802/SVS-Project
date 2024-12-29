import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
import numpy as np
from collections import deque
import carla_utility, time

class PID:
    def __init__(self, dt, buffer_size, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.e_buffer = deque(maxlen=buffer_size) if buffer_size != None and buffer_size > 0 else deque()

    def compute_control(self, target, current):
        e = target - current
        self.e_buffer.append(e)
        ie = np.sum(self.e_buffer) * self.dt
        de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt if len(self.e_buffer) > 1 else 0.0

        return np.clip((self.kp * e) + (self.kd * de) + (self.ki * ie), 0.0, 1.0)
    
class PIDController:
    def __init__(self, tc, buffer_size = 42, kp_velocity = 0.4, ki_velocity = 0.2, kd_velocity = 0.01, 
                kp_distance = 1.6, ki_distance = 0.01, kd_distance = 0.7):
        self.pid_velocity = PID(tc, buffer_size, kp_velocity, ki_velocity, kd_velocity)
        self.pid_distance = PID(tc, buffer_size, kp_distance, ki_distance, kd_distance)
        self.update_frequency = tc
        self.last_update = 0

    def __reset(self):
        self.last_throttle = 0 
        self.last_brake = 0 

    def apply_control(self, control_info, current_velocity, min_depth):
        if control_info.reset:
            self.__reset()
            control_info.reset = False
            
        if(time.time() - self.last_update > self.update_frequency):
            min_permitted_distance = carla_utility.compute_security_distance(current_velocity) + control_info.min_permitted_offset
            distance_error = min_depth - min_permitted_distance
            if distance_error > 0:
                self.last_throttle = self.pid_velocity.compute_control(control_info.target_velocity, current_velocity)
                control_info.ego_control.throttle = self.last_throttle
            else:
                self.last_brake = self.pid_distance.compute_control(min_permitted_distance, min_depth)
                control_info.ego_control.brake = self.last_brake

            self.last_update = time.time()
        else:
            control_info.ego_control.throttle = self.last_throttle
            control_info.ego_control.brake = self.last_brake