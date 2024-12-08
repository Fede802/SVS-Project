import numpy as np
from collections import deque
import time



class PID:
    def __init__(self, tc, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = tc
        self.e_buffer = deque(maxlen=2)
        self.last_update_time = time.time()

    def compute_control(self, target, current):
        """
        Compute the PID control based on PID equations.
        
        :param target: Target value
        :param current: Current value
        :return: Control value
        """
        e = target - current
        self.e_buffer.append(e)
        self.dt = time.time() - self.last_update_time
        self.last_update_time = time.time()
        if len(self.e_buffer) >= 2:
            de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt
            ie = sum(self.e_buffer) * self.dt
        else:
            de = 0.0
            ie = 0.0

        return np.clip(
            (self.kp * e) + (self.kd * de) + (self.ki * ie),
            -1.0,
            1.0
        )
    
class PIDController:
    def __init__(self, tc,  kp_speed = 0.4, ki_speed = 0.2, kd_speed = 0.01, 
                kp_distance = 1.6, ki_distance = 0.01, kd_distance = 0.7):
        self.pid_speed = PID(tc, kp_speed, ki_speed, kd_speed)
        self.pid_distance = PID(tc, kp_distance, ki_distance, kd_distance)

    def compute_pid_control_speed(self, target_speed, current_speed):
        """
        Compute the throttle control based on PID equations for speed.
        
        :param target_speed: Target speed in Km/h
        :param current_speed: Current speed of the vehicle in Km/h
        :return: Throttle control in the range [0, 1]
        """
        return self.pid_speed.compute_control(target_speed, current_speed)

    def compute_pid_control_distance(self, min_permitted_distance, current_distance):
        """
        Compute the brake control based on PID equations for distance.
        
        :param min_permitted_distance: Minimum permitted distance from the vehicle in front
        :param current_distance: Current distance from the vehicle in front
        :return: Brake control in the range [0, 1]
        """
        return self.pid_distance.compute_control(min_permitted_distance, current_distance)
    