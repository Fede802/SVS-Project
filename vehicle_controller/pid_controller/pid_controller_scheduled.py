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
        """
        Compute the PID control based on PID equations.
        
        :param target: Target value
        :param current: Current value
        :return: Control value
        """
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

    def apply_control(self, control_info, current_velocity, min_depth):
        if(time.time() - self.last_update > self.update_frequency):
            min_permitted_distance = carla_utility.compute_security_distance(current_velocity) + control_info.min_permitted_offset
            distance_error = min_depth - min_permitted_distance
            
            if distance_error > 0:
                control_info.ego_control.throttle = self.__compute_pid_control_velocity(control_info.target_velocity, current_velocity)
                control_info.ego_control.brake = 0
            else:
                control_info.ego_control.brake = self.__compute_pid_control_distance(min_permitted_distance, min_depth)
                control_info.ego_control.throttle = 0
            self.last_update = time.time()      

    def __compute_pid_control_velocity(self, target_velocity, current_velocity):
        """
        Compute the throttle control based on PID equations for velocity.
        
        :param target_velocity: Target velocity in Km/h
        :param current_velocity: Current velocity of the vehicle in Km/h
        :return: Throttle control in the range [0, 1]
        """
        return self.pid_velocity.compute_control(target_velocity, current_velocity)

    def __compute_pid_control_distance(self, min_permitted_distance, current_distance):
        """
        Compute the brake control based on PID equations for distance.
        
        :param min_permitted_distance: Minimum permitted distance from the vehicle in front
        :param current_distance: Current distance from the vehicle in front
        :return: Brake control in the range [0, 1]
        """
        return self.pid_distance.compute_control(min_permitted_distance, current_distance)
    